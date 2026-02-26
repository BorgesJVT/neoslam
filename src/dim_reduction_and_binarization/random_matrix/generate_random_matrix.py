#!/usr/bin/env python3
"""
Generate random projection matrix directly in binary format.

Creates a random matrix for LSBH (Locality-Sensitive Binary Hashing) and saves
it directly as binary file.

Matrix dimensions:
- Rows: 64896 (AlexNet Conv3 output: 384 x 13 x 13)
- Cols: 1024 (target dimensionality for random projection)

Output: randomMatrix.bin (raw float32 binary data, row-major)
"""

import numpy as np
import os
import argparse

def generate_random_matrix_binary(rows, cols, output_path, seed=None):
    """
    Generate random projection matrix and save as binary file.
    
    Args:
        rows: Number of rows (input feature dimension)
        cols: Number of columns (projection dimension)
        output_path: Path to output .bin file
        seed: Random seed for reproducibility (optional)
    """
    print(f"Generating random projection matrix...")
    print(f"  Dimensions: {rows} x {cols}")
    print(f"  Seed: {seed if seed is not None else 'random'}")
    
    # Set random seed for reproducibility
    if seed is not None:
        np.random.seed(seed)
    
    # Generate random matrix with normal distribution (standard for random projection)
    # Using float32 to match C++ float type
    random_matrix = np.random.randn(rows, cols).astype(np.float32)
    
    # Normalize columns (optional, helps with numerical stability)
    # Uncomment if you want normalized projections:
    # random_matrix = random_matrix / np.linalg.norm(random_matrix, axis=0, keepdims=True)
    
    print(f"  Data type: {random_matrix.dtype}")
    print(f"  Memory order: C-contiguous (row-major)")
    print(f"  Size: {random_matrix.nbytes / 1024 / 1024:.2f} MB")
    print(f"  Min value: {random_matrix.min():.6f}")
    print(f"  Max value: {random_matrix.max():.6f}")
    print(f"  Mean: {random_matrix.mean():.6f}")
    print(f"  Std: {random_matrix.std():.6f}")
    
    # Ensure directory exists
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"  Created directory: {output_dir}")
    
    # Save as binary file (row-major order)
    print(f"\nSaving to {output_path}...")
    random_matrix.tofile(output_path)
    
    # Verify file was created
    if os.path.exists(output_path):
        file_size_mb = os.path.getsize(output_path) / 1024 / 1024
        print(f"✓ Successfully created: {output_path}")
        print(f"  File size: {file_size_mb:.2f} MB")
        
        # Verification: read back and check
        print(f"\nVerifying file integrity...")
        loaded_matrix = np.fromfile(output_path, dtype=np.float32).reshape(rows, cols)
        if np.allclose(random_matrix, loaded_matrix):
            print(f"✓ Verification passed: file can be read correctly")
        else:
            print(f"✗ WARNING: Verification failed - file may be corrupted")
            return False
    else:
        print(f"✗ ERROR: Failed to create file")
        return False
    
    return True

def main():
    parser = argparse.ArgumentParser(
        description='Generate random projection matrix in binary format for LSBH',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate default matrix (64896 x 1024)
  python generate_random_matrix.py
  
  # Custom output path
  python generate_random_matrix.py -o /path/to/matrix.bin
  
  # Custom dimensions
  python generate_random_matrix.py -r 1000 -c 500
  
  # With specific seed for reproducibility
  python generate_random_matrix.py --seed 42
        """
    )
    
    parser.add_argument('-r', '--rows', type=int, default=64896,
                        help='Number of rows (default: 64896 for AlexNet Conv3)')
    parser.add_argument('-c', '--cols', type=int, default=1024,
                        help='Number of columns (default: 1024)')
    parser.add_argument('-o', '--output', type=str, default='random_matrix/randomMatrix.bin',
                        help='Output binary file path (default: random_matrix/randomMatrix.bin)')
    parser.add_argument('--seed', type=int, default=None,
                        help='Random seed for reproducibility (default: random)')
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("Random Projection Matrix Generator")
    print("=" * 70)
    
    success = generate_random_matrix_binary(
        rows=args.rows,
        cols=args.cols,
        output_path=args.output,
        seed=args.seed
    )
    
    if success:
        print("✓ Matrix generation complete!")
        print(f"\nMatrix will be loaded as: ({args.rows}, {args.cols}) float32")
    else:
        print("\n✗ Matrix generation failed!")
        return 1
    
    return 0

if __name__ == "__main__":
    import sys
    sys.exit(main())
