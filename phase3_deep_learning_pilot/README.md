# Phase 3: Deep Learning Pilot with Faster R-CNN

**Period**: August 2023  
**Context**: Independent research initiative exploring ML approaches for protein band detection

## Research Motivation
Inspired by a machine learning course, I proposed investigating whether deep learning could:
1. Match hand-crafted algorithm accuracy while leveraging GPU acceleration
2. Improve generalization to unseen gel variations
3. Validate the "band-first" paradigm in a modern ML framework

## Project Overview
Under mentorship of a senior computer vision engineer, I independently developed an end-to-end protein band detector based on Faster R-CNN, comparing performance against my hand-crafted C++ algorithms.

## Methodology

### 1. Data Preparation Pipeline
Dataset Statistics:
- Total Images: 200+ gel electrophoresis images
- Annotations: 2,847 protein band bounding boxes
- Tool: LabelImg for manual annotation
- Format: Converted to COCO standard
- Quality Control: Senior engineer reviewed edge cases

Annotation Protocol:
- Single annotator (myself) for consistency
- Bounding boxes around all visible protein bands
- Edge case guidelines: Overlapping bands, faint bands, irregular shapes
- COCO format conversion for TensorFlow compatibility

### 2. Model Development

#### Architecture Selection
Model: Faster R-CNN with ResNet-50 backbone

Rationale:
1. Two-stage detector suitable for medium-sized objects
2. COCO pretraining provides strong feature representations
3. Balance between accuracy and computational efficiency

#### Domain-Specific Customization

Anchor Box Optimization via K-means Clustering:

1. Extract width/height from all annotated bounding boxes

2. Apply K-means clustering (k=9) on box dimensions

3. Convert cluster centers to anchor scales and aspect ratios

4. Result: Anchors optimized for elongated protein band shapes

5. #### Training Configuration
   Framework: TensorFlow Object Detection API
   Pretraining: COCO dataset weights
   Strategy: Transfer learning with frozen backbone

   Hyperparameters:
   - Batch Size: 8 (GPU memory constrained)
   - Learning Rate: 0.0001 with cosine decay
   - Epochs: 50 with early stopping
   - Augmentation: Random flip (±5° rotation)

   ### 3. Experimental Design

   #### Baseline Comparison
   - Comparison Algorithm: Production C++ algorithm from Phase 2
   - Evaluation Metrics: Precision, Recall, F1-Score, IoU, Inference Time
   - Test Set: 32 held-out gel images (15% of total)

   #### Hardware Configuration
   CPU Baseline: Intel Xeon (C++ algorithm execution)
   GPU Acceleration: NVIDIA Tesla V100 (TensorFlow execution)

   ## Results & Analysis

   ### Quantitative Performance Comparison
   | Metric    | C++ Algorithm (Phase 2) | Faster R-CNN  | Statistical Significance      |
   | --------- | ----------------------- | ------------- | ----------------------------- |
   | Precision | 92.3% ± 2.1%            | 91.8% ± 2.4%  | p = 0.42 (not significant)    |
   | Recall    | 90.7% ± 2.8%            | 91.2% ± 2.6%  | p = 0.38 (not significant)    |
   | F1-Score  | 91.5% ± 1.9%            | 91.5% ± 2.0%  | Equivalent performance        |
   | IoU       | 0.892 ± 0.043           | 0.873 ± 0.051 | Slight degradation (p = 0.03) |

   ### Inference Speed Analysis
   | Hardware      | C++ Algorithm   | Faster R-CNN | Speed Ratio   |
   | ------------- | --------------- | ------------ | ------------- |
   | CPU Execution | 1.0× (baseline) | 3.2×         | 220% slower   |
   | GPU Execution | N/A             | 0.3-0.5×     | 50-70% faster |

   ### Key Findings
   1. Accuracy Parity: Deep learning achieved statistically equivalent accuracy to carefully engineered algorithms
   2. GPU Acceleration: 50-70% faster inference when leveraging GPU hardware
   3. Hardware Dependency: CPU inference significantly slower, highlighting GPU requirement
   4. Generalization: Better handling of extreme gel variations not seen in training

   ## Technical Contributions

   ### 1. End-to-End Pipeline Development
   - Data annotation → Model training → Evaluation complete workflow
   - Reproducible experimentation with detailed documentation
   - Comparative analysis framework for algorithm evaluation

   ### 2. Domain-Specific Innovations
   - K-means anchor optimization for elongated protein bands
   - Transfer learning strategy with frozen backbone
   - Comprehensive evaluation against production baseline

   ### 3. Research Methodology
   - Hypothesis-driven experimentation: Can DL match hand-crafted accuracy?
   - Rigorous comparison: Direct benchmarking with existing algorithm
   - Quantitative validation: Statistical analysis of performance differences

   ## Business & Research Implications

   ### Practical Considerations
   1. GPU Requirement: Speed advantage contingent on GPU availability
   2. Data Requirements: 200+ annotated images for training
   3. Deployment Complexity: TensorFlow vs. lightweight C++ deployment

   ### Research Directions
   1. Lightweight Models: MobileNet variants for CPU deployment
   2. Semi-Supervised Learning: Reducing annotation requirements
   3. Hybrid Approaches: Combining traditional CV with deep learning

   ## Conclusion
   This pilot project demonstrated that:
   1. The "band-first" paradigm translates effectively to deep learning
   2. Accuracy parity is achievable with sufficient training data
   3. GPU acceleration provides significant inference speed advantages
   4. Hand-crafted algorithms remain valuable for CPU deployment scenarios

   The work bridges classical computer vision with modern deep learning, providing a comprehensive comparison of approaches for scientific image analysis.

   ---
   Part of a three-phase research journey: [Phase 1](../phase1_band_first_algorithm) • [Phase 2](../phase2_algorithm_optimization)
