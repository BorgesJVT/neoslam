#pragma once
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>
#include <roaring/roaring.hh>
#include <vector>
#include <memory>
#include <chrono>
#include <iostream>
#include <algorithm>

/**
 * @file local_view_match_with_intervals.hpp
 * @brief Versão melhorada e otimizada do ViewCells usando terminologia RatSLAM
 * 
 * OBJETIVO:
 * Substituir ViewCells mantendo a mesma interface e semântica, mas com
 * terminologia inspirada no RatSLAM (Visual Templates ao invés de ViewCells).
 * 
 * MELHORIAS IMPLEMENTADAS:
 * 1. Terminologia mais clara inspirada no RatSLAM
 * 2. Código mais legível e documentado
 * 3. Mesma estrutura de intervalos otimizada
 * 4. Interface simplificada: retorna int ID diretamente
 * 5. Máxima simplificação: usa apenas IDs int, sem classes wrapper
 * 
 * MUDANÇA DE INTERFACE:
 * - Função on_image_map agora retorna std::pair<Eigen::MatrixXd, int>
 * - Segundo elemento é o ID do visual template (int) ao invés de ViewCell
 * - Mais simples, direto e eficiente!
 */

/**
 * @brief TemplateInterval agrupa visual templates temporalmente próximos e similares
 * 
 * Equivalente semântico do Interval original, mas com nomenclatura mais clara.
 * Reduz comparações ao agrupar imagens similares em intervalos contíguos.
 */
class TemplateInterval {
public:
    std::pair<int, int> init_end;          // [início, fim] índices de imagens
    Roaring accumulated_features;           // União de todas as features do intervalo
    
    TemplateInterval() : init_end{0, 0} {}
};

/**
 * @brief LocalViewMatchWithIntervals - Versão melhorada do ViewCells
 * 
 * Esta classe implementa a mesma lógica do ViewCells original, mas com:
 * - Terminologia inspirada no RatSLAM (mais intuitiva)
 * - Código mais limpo e documentado
 * - Mesma estrutura de intervalos otimizada com Roaring Bitmaps
 * - Interface simplificada: retorna int ID ao invés de objeto ViewCell
 * - Máxima simplificação: usa apenas vector<int> para IDs dos visual templates
 * 
 * TERMINOLOGIA:
 * - Visual Template ID = int (lugares únicos reconhecidos)
 * - TemplateInterval = agrupa imagens similares
 * - on_image_map → retorna pair<MatrixXd, int>
 * 
 * USO NO SIMULATION.CPP:
 * Substituir:
 *   #include "viewcells.hpp"
 *   lv_ = std::make_unique<ViewCells>(...);
 *   auto [interval_map, cell] = lv_->on_image_map(d1_htm_roaring, counter_);
 *   int id = cell.id;
 * 
 * Por:
 *   #include "local_view_match_with_intervals.hpp"
 *   lv_ = std::make_unique<LocalViewMatchWithIntervals>(...);
 *   auto [interval_map, template_id] = lv_->on_image_map(d1_htm_roaring, counter_);
 *   // template_id já é int diretamente!
 * 
 * MUDANÇA: Retorno simplificado - int ao invés de ViewCell
 */
class LocalViewMatchWithIntervals {
public:
    /**
     * @brief Constructor - mesmos parâmetros do ViewCells original
     * 
     * @param theta_alpha_ Threshold de similaridade para estender intervalos
     * @param theta_rho_ Duração máxima de um intervalo (em imagens)
     * @param score_interval_ Threshold de score para detecção de loop closure
     * @param exclude_recent_intervals_ Número de intervalos recentes a excluir da detecção de loop closure
     */
    LocalViewMatchWithIntervals(int theta_alpha_, int theta_rho_, int score_interval_, 
                                int exclude_recent_intervals_ = 3)
        : theta_alpha(theta_alpha_), 
          theta_rho(theta_rho_), 
          score_interval(score_interval_),
          exclude_recent_intervals(exclude_recent_intervals_),
          is_initialized_(false),  // Ainda não processou primeira imagem
          n_interval(0), 
          prev_interval(0),
          current_template_id(-1),
          next_template_id(0)  // Contador para gerar novos IDs
    {
        // std::cout << "[LocalViewMatchWithIntervals] Initialized with:" << std::endl;
        // std::cout << "  theta_alpha = " << theta_alpha << " (similarity threshold)" << std::endl;
        // std::cout << "  theta_rho = " << theta_rho << " (max interval duration)" << std::endl;
        // std::cout << "  score_interval = " << score_interval << " (loop closure threshold)" << std::endl;
        // std::cout << "  exclude_recent_intervals = " << exclude_recent_intervals << " (recent intervals to exclude)" << std::endl;
    }

    /**
     * @brief Cria um novo Visual Template ID (lugar único)
     * Simplificação máxima: apenas incrementa contador e armazena ID
     */
    int create_template_id() {
        int new_id = next_template_id++;
        visual_template_ids.push_back(new_id);
        return new_id;
    }

    /**
     * @brief Função principal - processa nova imagem e detecta loop closures
     * 
     * INTERFACE SIMPLIFICADA: Retorna int ID diretamente!
     * 
     * PIPELINE:
     * 1. Gerenciamento de intervalos (criar/estender)
     * 2. Construção incremental do mapa de intervalos
     * 3. Loop closure detection (comparação com todos intervalos)
     * 4. Decisão: criar novo visual template ou reutilizar existente
     * 
     * @param feature Roaring Bitmap com features ativas (512 bits típico)
     * @param n_image Índice da imagem atual (contador)
     * @return std::pair<Eigen::MatrixXd, int> - (scores, template_id)
     *         - scores: vetor com similaridades para cada intervalo
     *         - template_id: ID do visual template (int) reconhecido ou criado
     */
    std::pair<Eigen::MatrixXd, int> on_image_map(const Roaring& feature, int n_image) {
        auto overall_start = std::chrono::high_resolution_clock::now();
        
        // ==============================================================
        // INICIALIZAÇÃO: Primeira imagem (executado uma única vez)
        // ==============================================================
        if (!is_initialized_) {
            initialize_first_template(feature, n_image);
            is_initialized_ = true;
        }
        
        // ==============================================================
        // PASSO 1: Informações da feature recebida
        // ==============================================================
        int template_id;  // ID do visual template (simplificado!)
        // std::cout << "n_image: " << n_image << std::endl;
        // std::cout << "  [Roaring] Feature has " << feature.cardinality() << " active bits" << std::endl;
        
        // ==============================================================
        // PASSO 2: Gerenciamento de Intervalos Temporais
        // Agrupa imagens similares para reduzir comparações
        // ==============================================================
        
        // Calcula similaridade com intervalo atual
        Roaring& anchor = interval_list[n_interval]->accumulated_features;
        int alpha = (anchor & feature).cardinality();  // Interseção bitwise
        
        // std::cout << "  [Similarity] alpha=" << alpha 
        //           << " (threshold=" << theta_alpha << ")" << std::endl;
        
        // Calcula duração do intervalo atual
        int interval_duration = interval_list[n_interval]->init_end.second 
                              - interval_list[n_interval]->init_end.first;
        
        // Decisão: estender ou criar novo?
        if ((alpha >= theta_alpha) && (interval_duration < theta_rho)) {
            // ==========================================
            // Estende intervalo atual (imagem similar)
            // ==========================================
            interval_list[n_interval]->init_end.second += 1;
            interval_list[n_interval]->accumulated_features |= feature;  // União (OR)
            
            // std::cout << "  [Interval] EXTENDED interval " << n_interval 
            //           << " (duration=" << (interval_duration + 1) << ")" << std::endl;
        } else {
            // ==========================================
            // Cria novo intervalo (dissimilar ou muito longo)
            // ==========================================
            current_interval = std::make_shared<TemplateInterval>();
            current_interval->init_end = {n_image, n_image};
            current_interval->accumulated_features = feature;
            interval_list.push_back(current_interval);
            n_interval += 1;
            
            std::string reason = (alpha < theta_alpha) ? "low_similarity" : "max_duration";
            // std::cout << "  [Interval] Created NEW interval " << n_interval 
            //           << " (reason=" << reason << ")" << std::endl;
        }
        
        // std::cout << "  [State] n_interval=" << n_interval 
        //           << ", total_intervals=" << interval_list.size() << std::endl;
        
        // ==============================================================
        // PASSO 3: Construção Incremental do Mapa de Intervalos
        // Usa vector de Roaring Bitmaps (extremamente eficiente!)
        // ==============================================================
        auto map_build_start = std::chrono::high_resolution_clock::now();
        
        // std::cout << "  [Map] intervals_feature_map.size=" << intervals_feature_map.size() << std::endl;
        
        if (prev_interval != n_interval) {
            // Novo intervalo: adiciona nova linha ao mapa
            intervals_feature_map.push_back(interval_list[n_interval]->accumulated_features);
            
            // std::cout << "  [Map] NEW interval row added (n_interval=" << n_interval 
            //           << ", total_rows=" << intervals_feature_map.size() 
            //           << ", active_bits=" << intervals_feature_map.back().cardinality() << ")" << std::endl;
        } else {
            // Mesmo intervalo: atualiza última linha com features acumuladas
            if (!intervals_feature_map.empty()) {
                intervals_feature_map.back() = interval_list[n_interval]->accumulated_features;
                
                // std::cout << "  [Map] SAME interval updated (n_interval=" << n_interval 
                //           << ", active_bits=" << intervals_feature_map.back().cardinality() << ")" << std::endl;
            }
        }
        
        auto map_build_end = std::chrono::high_resolution_clock::now();
        auto map_build_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            map_build_end - map_build_start);
        // std::cout << "  [TIMING] Map rebuild: " << map_build_duration.count() / 1000.0 << " ms" << std::endl;
        
        // ==============================================================
        // PASSO 4: Loop Closure Detection
        // Calcula similaridade com TODOS os intervalos passados
        // Usa operações bitwise SIMD-otimizadas (AND + popcount)
        // ==============================================================
        auto loop_closure_start = std::chrono::high_resolution_clock::now();
        
        // std::cout << "  [Loop Closure] Computing similarity with " 
        //           << intervals_feature_map.size() << " intervals..." << std::endl;
        
        // Calcula scores de similaridade para cada intervalo
        int num_intervals = intervals_feature_map.size();
        Eigen::VectorXi similarity_scores(num_intervals);
        
        for (int i = 0; i < num_intervals; ++i) {
            // Interseção bitwise + contagem (SIMD otimizado)
            similarity_scores(i) = (intervals_feature_map[i] & feature).cardinality();
        }
        
        auto loop_closure_end = std::chrono::high_resolution_clock::now();
        auto loop_closure_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            loop_closure_end - loop_closure_start);
        // std::cout << "  [TIMING] Loop closure computation: " 
        //           << loop_closure_duration.count() / 1000.0 << " ms" << std::endl;
        
        // ==============================================================
        // PASSO 5: Converte scores para formato de retorno (MatrixXd)
        // ==============================================================
        Eigen::MatrixXd interval_map = similarity_scores.cast<double>();
        
        // ==============================================================
        // PASSO 6: Filtragem de Candidatos a Loop Closure
        // Exclui últimos N intervalos (muito recentes, evita falsos positivos)
        // ==============================================================
        Eigen::Index n = similarity_scores.size();
        Eigen::Index block_rows = n > exclude_recent_intervals ? n - exclude_recent_intervals : 0;
        Eigen::VectorXi scores_for_matching;
        
        if (block_rows > 0) {
            scores_for_matching = similarity_scores.head(block_rows);
        } else {
            scores_for_matching = Eigen::VectorXi();  // Vazio
        }
        
        // ==============================================================
        // PASSO 7: Decisão - Criar Novo Template ou Reutilizar Existente
        // ==============================================================
        
        // ==========================================
        // Busca por loop closures
        // ==========================================
        std::vector<int> matched_intervals;
        for (int i = 0; i < scores_for_matching.size(); ++i) {
            if (scores_for_matching(i) > score_interval) {
                matched_intervals.push_back(i);
            }
        }
        
        if (matched_intervals.empty()) {
            // ==========================================
            // Nenhum match: lugar novo
            // ==========================================
            if (prev_interval != n_interval) {
                // Novo intervalo: cria novo template
                template_id = create_template_id();
                // std::cout << "  [Visual Template] Created NEW template (id=" 
                //           << template_id << ", no_match)" << std::endl;
            } else {
                // Mesmo intervalo: reutiliza template anterior
                template_id = current_template_id;
                // std::cout << "  [Visual Template] Reusing previous template (id=" 
                //           << template_id << ", same_interval)" << std::endl;
            }
        } else {
            // ==========================================
            // LOOP CLOSURE DETECTADO!
            // Reutiliza template do intervalo mais similar
            // ==========================================
            
            // Encontra intervalo com maior score
            int best_interval_idx = std::distance(
                scores_for_matching.data(), 
                std::max_element(scores_for_matching.data(), 
                               scores_for_matching.data() + scores_for_matching.size())
            );
            
            template_id = interval_to_template_map[best_interval_idx];
            
            // std::cout << "  [Visual Template] LOOP CLOSURE! Reusing template " 
            //           << template_id 
            //           << " (interval=" << best_interval_idx 
            //           << ", similarity=" << scores_for_matching(best_interval_idx) 
            //           << ")" << std::endl;
        }
        
        // ==============================================================
        // PASSO 8: Atualização de Estado para Próxima Iteração
        // ==============================================================
        if (prev_interval != n_interval) {
            interval_to_template_map.push_back(template_id);
        }
        current_template_id = template_id;
        prev_interval = n_interval;
        
        // ==============================================================
        // PASSO 9: Finalização e Métricas
        // ==============================================================
        auto overall_end = std::chrono::high_resolution_clock::now();
        auto overall_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            overall_end - overall_start);
        // std::cout << "  [TIMING] Total on_image_map: " 
        //           << overall_duration.count() / 1000.0 << " ms" << std::endl;
        
        // Retorna: (scores_de_similaridade, template_id)
        // INTERFACE SIMPLIFICADA: Retorna int diretamente!
        return std::make_pair(interval_map, template_id);
    }

    // ==============================================================
    // Métodos auxiliares públicos para debugging e análise
    // ==============================================================
    
    /**
     * @brief Retorna número total de visual templates (lugares reconhecidos)
     */
    size_t get_num_templates() const { 
        return visual_template_ids.size(); 
    }
    
    /**
     * @brief Retorna número total de intervalos criados
     */
    size_t get_num_intervals() const { 
        return interval_list.size(); 
    }
    
    /**
     * @brief Retorna índice do intervalo atual
     */
    int get_current_interval() const { 
        return n_interval; 
    }
    
    /**
     * @brief Retorna ID do visual template atual
     */
    int get_current_template_id() const { 
        return current_template_id; 
    }
    
    /**
     * @brief Retorna lista de todos os IDs de visual templates criados
     */
    const std::vector<int>& get_template_ids() const {
        return visual_template_ids;
    }
    
    /**
     * @brief Calcula scores de similaridade para uma feature específica
     * Útil para análise e visualização
     */
    Eigen::VectorXi get_interval_similarity_scores(const Roaring& feature) const {
        int num_intervals = intervals_feature_map.size();
        Eigen::VectorXi scores(num_intervals);
        for (int i = 0; i < num_intervals; ++i) {
            scores(i) = (intervals_feature_map[i] & feature).cardinality();
        }
        return scores;
    }
    
    /**
     * @brief Retorna informações sobre um intervalo específico
     */
    std::pair<int, int> get_interval_range(int interval_id) const {
        if (interval_id >= 0 && interval_id < (int)interval_list.size()) {
            return interval_list[interval_id]->init_end;
        }
        return {-1, -1};
    }
    
    /**
     * @brief Retorna número de features ativas em um intervalo
     */
    uint64_t get_interval_feature_count(int interval_id) const {
        if (interval_id >= 0 && interval_id < (int)intervals_feature_map.size()) {
            return intervals_feature_map[interval_id].cardinality();
        }
        return 0;
    }

private:
    // ==============================================================
    // Parâmetros de Configuração
    // ==============================================================
    int theta_alpha;       // Threshold de similaridade para intervalos
    int theta_rho;         // Duração máxima de um intervalo
    int score_interval;    // Threshold para detecção de loop closure
    int exclude_recent_intervals;  // Número de intervalos recentes a excluir
    
    // ==============================================================
    // Estado Interno
    // ==============================================================
    bool is_initialized_;      // Flag de inicialização (primeira imagem processada?)
    int n_interval;            // Índice do intervalo atual
    int prev_interval;         // Índice do intervalo anterior
    int current_template_id;   // ID do visual template atual
    int next_template_id;      // Contador para gerar novos IDs
    
    // ==============================================================
    // Estruturas de Dados
    // ==============================================================
    
    // Lista de IDs de visual templates (lugares únicos) - SIMPLIFICAÇÃO MÁXIMA!
    // Apenas armazena os IDs, não objetos completos
    std::vector<int> visual_template_ids;
    
    // Lista de intervalos temporais
    std::vector<std::shared_ptr<TemplateInterval>> interval_list;
    
    // Mapeamento: intervalo_id → visual_template_id
    std::vector<int> interval_to_template_map;
    
    // Ponteiro para intervalo atual
    std::shared_ptr<TemplateInterval> current_interval;
    
    // ==========================================
    // OTIMIZAÇÃO CHAVE: Vector de Roaring Bitmaps
    // ==========================================
    // Cada elemento representa features acumuladas de um intervalo
    // Memória: ~2 KB por intervalo (vs 4-8 KB com estruturas densas)
    // Performance: operações bitwise SIMD nativas (AVX2/AVX-512)
    std::vector<Roaring> intervals_feature_map;
    
    // ==============================================================
    // Método privado de inicialização
    // ==============================================================
    
    /**
     * @brief Inicializa estruturas com a primeira imagem
     * Chamado automaticamente na primeira execução de on_image_map
     */
    void initialize_first_template(const Roaring& feature, int n_image) {
        // Cria primeiro intervalo
        current_interval = std::make_shared<TemplateInterval>();
        current_interval->init_end = {n_image, n_image};
        current_interval->accumulated_features = feature;
        interval_list.push_back(current_interval);
        
        // Cria primeiro visual template
        int first_template_id = create_template_id();
        interval_to_template_map.push_back(first_template_id);
        current_template_id = first_template_id;
    }
};