#!/usr/bin/env python3
"""
HSV Test Results Analyzer

This script analyzes the CSV output from the HSV test tool and provides
insights for optimal parameter selection.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import glob
import os
import sys
from datetime import datetime

def load_latest_results(results_dir="/tmp/hsv_test_results"):
    """Load the most recent CSV results file"""
    csv_files = glob.glob(os.path.join(results_dir, "hsv_test_results_*.csv"))
    
    if not csv_files:
        print(f"No CSV files found in {results_dir}")
        return None
    
    # Get the most recent file
    latest_file = max(csv_files, key=os.path.getctime)
    print(f"Loading results from: {latest_file}")
    
    try:
        df = pd.read_csv(latest_file)
        print(f"Loaded {len(df)} test results")
        return df, latest_file
    except Exception as e:
        print(f"Error loading CSV: {e}")
        return None, None

def analyze_color_coverage(df):
    """Analyze color coverage statistics"""
    colors = ['red', 'yellow', 'green', 'blue', 'grey']
    
    print("\n=== COLOR COVERAGE ANALYSIS ===")
    
    for color in colors:
        percentage_col = f"{color}_percentage"
        if percentage_col in df.columns:
            stats = df[percentage_col].describe()
            print(f"\n{color.upper()} Coverage:")
            print(f"  Mean: {stats['mean']:.2f}%")
            print(f"  Std:  {stats['std']:.2f}%")
            print(f"  Min:  {stats['min']:.2f}%")
            print(f"  Max:  {stats['max']:.2f}%")

def analyze_successful_captures(df):
    """Analyze parameters that led to successful captures"""
    if 'board_capture_success' not in df.columns:
        print("No board capture data available")
        return
    
    print("\n=== SUCCESSFUL CAPTURE ANALYSIS ===")
    
    successful = df[df['board_capture_success'] == True]
    failed = df[df['board_capture_success'] == False]
    
    print(f"Successful captures: {len(successful)}/{len(df)} ({len(successful)/len(df)*100:.1f}%)")
    
    if len(successful) == 0:
        print("No successful captures to analyze")
        return
    
    # Analyze HSV parameters for successful captures
    colors = ['red1', 'red2', 'yellow', 'green', 'blue', 'grey']
    hsv_params = ['h_low', 'h_high', 's_low', 's_high', 'v_low', 'v_high']
    
    print("\nOptimal HSV ranges for successful captures:")
    
    for color in colors:
        print(f"\n{color.upper()}:")
        for param in hsv_params:
            col_name = f"{color}_{param}"
            if col_name in successful.columns:
                values = successful[col_name]
                print(f"  {param:7}: {values.min():3d} - {values.max():3d} (mean: {values.mean():.1f})")

def analyze_lighting_conditions(df):
    """Analyze performance under different lighting conditions"""
    if 'lighting_condition' not in df.columns:
        return
    
    print("\n=== LIGHTING CONDITION ANALYSIS ===")
    
    lighting_groups = df.groupby('lighting_condition')
    
    for condition, group in lighting_groups:
        if condition and condition.strip():  # Skip empty conditions
            success_rate = 0
            if 'board_capture_success' in group.columns:
                success_rate = group['board_capture_success'].mean() * 100
            
            print(f"\n{condition}:")
            print(f"  Tests: {len(group)}")
            print(f"  Success rate: {success_rate:.1f}%")
            
            # Show average color coverage
            colors = ['red', 'yellow', 'green', 'blue', 'grey']
            for color in colors:
                percentage_col = f"{color}_percentage"
                if percentage_col in group.columns:
                    avg_coverage = group[percentage_col].mean()
                    print(f"  {color:6} coverage: {avg_coverage:.1f}%")

def create_visualizations(df, output_dir):
    """Create visualization plots"""
    try:
        import matplotlib.pyplot as plt
        import seaborn as sns
        
        plt.style.use('default')
        fig_dir = os.path.join(output_dir, 'analysis_plots')
        os.makedirs(fig_dir, exist_ok=True)
        
        # Color coverage distribution
        colors = ['red', 'yellow', 'green', 'blue', 'grey']
        coverage_data = []
        
        for color in colors:
            percentage_col = f"{color}_percentage"
            if percentage_col in df.columns:
                coverage_data.extend([(color, val) for val in df[percentage_col]])
        
        if coverage_data:
            coverage_df = pd.DataFrame(coverage_data, columns=['Color', 'Coverage'])
            
            plt.figure(figsize=(10, 6))
            sns.boxplot(data=coverage_df, x='Color', y='Coverage')
            plt.title('Color Coverage Distribution')
            plt.ylabel('Coverage (%)')
            plt.xticks(rotation=45)
            plt.tight_layout()
            plt.savefig(os.path.join(fig_dir, 'color_coverage_distribution.png'))
            plt.close()
        
        # Success rate over time
        if 'board_capture_success' in df.columns and len(df) > 1:
            df['test_id_num'] = pd.to_numeric(df['test_id'], errors='coerce')
            df_sorted = df.sort_values('test_id_num')
            
            # Rolling success rate
            window_size = min(5, len(df_sorted))
            rolling_success = df_sorted['board_capture_success'].rolling(window=window_size, min_periods=1).mean()
            
            plt.figure(figsize=(12, 6))
            plt.plot(df_sorted['test_id_num'], rolling_success * 100, marker='o')
            plt.title(f'Success Rate Over Time (Rolling Average, Window={window_size})')
            plt.xlabel('Test ID')
            plt.ylabel('Success Rate (%)')
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.savefig(os.path.join(fig_dir, 'success_rate_over_time.png'))
            plt.close()
        
        # HSV parameter correlation with success (if enough data)
        if 'board_capture_success' in df.columns and len(df) > 10:
            hsv_cols = [col for col in df.columns if any(param in col for param in ['_h_', '_s_', '_v_'])]
            
            if hsv_cols:
                correlation_data = df[hsv_cols + ['board_capture_success']].corr()['board_capture_success'].drop('board_capture_success')
                
                plt.figure(figsize=(12, 8))
                correlation_data.plot(kind='bar')
                plt.title('HSV Parameter Correlation with Capture Success')
                plt.xlabel('HSV Parameters')
                plt.ylabel('Correlation')
                plt.xticks(rotation=90)
                plt.tight_layout()
                plt.savefig(os.path.join(fig_dir, 'hsv_success_correlation.png'))
                plt.close()
        
        print(f"\nVisualization plots saved to: {fig_dir}")
        
    except ImportError:
        print("Matplotlib/Seaborn not available for plotting")
    except Exception as e:
        print(f"Error creating visualizations: {e}")

def generate_recommendations(df):
    """Generate parameter recommendations based on analysis"""
    print("\n=== RECOMMENDATIONS ===")
    
    if 'board_capture_success' not in df.columns:
        print("No capture success data available for recommendations")
        return
    
    successful = df[df['board_capture_success'] == True]
    
    if len(successful) == 0:
        print("No successful captures found. Recommendations:")
        print("1. Check camera positioning and ArUco marker visibility")
        print("2. Improve lighting conditions")
        print("3. Adjust HSV ranges to better match your pieces")
        return
    
    print("Based on successful captures:")
    
    # Find most stable parameters (low std deviation in successful tests)
    colors = ['red1', 'red2', 'yellow', 'green', 'blue', 'grey']
    hsv_params = ['h_low', 'h_high', 's_low', 's_high', 'v_low', 'v_high']
    
    print("\nRecommended HSV ranges:")
    for color in colors:
        print(f"\n{color.upper()}:")
        for param in hsv_params:
            col_name = f"{color}_{param}"
            if col_name in successful.columns and len(successful) > 1:
                values = successful[col_name]
                mean_val = values.mean()
                std_val = values.std()
                
                # Recommend range: mean ± 1 std, bounded by observed min/max
                min_rec = max(0, int(mean_val - std_val))
                max_rec = min(255 if 's_' in param or 'v_' in param else 179, 
                             int(mean_val + std_val))
                
                print(f"  {param}: {min_rec} - {max_rec}")
    
    # Color coverage recommendations
    colors = ['red', 'yellow', 'green', 'blue']
    print("\nColor coverage analysis:")
    
    for color in colors:
        percentage_col = f"{color}_percentage"
        if percentage_col in successful.columns:
            avg_coverage = successful[percentage_col].mean()
            if avg_coverage < 5:
                print(f"- {color}: Low coverage ({avg_coverage:.1f}%). Consider widening HSV ranges.")
            elif avg_coverage > 30:
                print(f"- {color}: High coverage ({avg_coverage:.1f}%). Consider tightening HSV ranges.")
            else:
                print(f"- {color}: Good coverage ({avg_coverage:.1f}%)")

def main():
    """Main analysis function"""
    results_dir = "/tmp/hsv_test_results"
    
    # Allow custom directory as command line argument
    if len(sys.argv) > 1:
        results_dir = sys.argv[1]
    
    print(f"HSV Test Results Analyzer")
    print(f"Looking for results in: {results_dir}")
    
    result = load_latest_results(results_dir)
    if result is None:
        return
    
    df, csv_file = result
    
    if df is None or df.empty:
        print("No data to analyze")
        return
    
    # Run analyses
    analyze_color_coverage(df)
    analyze_successful_captures(df)
    analyze_lighting_conditions(df)
    
    # Create visualizations
    output_dir = os.path.dirname(csv_file)
    create_visualizations(df, output_dir)
    
    # Generate recommendations
    generate_recommendations(df)
    
    print(f"\nAnalysis complete. Results based on {len(df)} test samples.")
    print(f"Data source: {csv_file}")

if __name__ == "__main__":
    main()
