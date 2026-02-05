# Gel Electrophoresis Analysis: Three-Phase Research Journey

**Author**: Yue Yu | **USTC Statistics 2025**  
**Period**: 2022-2023 Internships at Biolight Biotech

## Overview
This repository documents my complete three-phase research journey in developing gel electrophoresis image analysis algorithms, demonstrating technical evolution from algorithmic innovation to deep learning exploration.

## Research Phases

### [Phase 1: Algorithm Innovation](/phase1_band_first_algorithm)
**July-August 2022** | **Complete C++ Implementation Available**
- **Problem**: Traditional algorithms fail completely on skewed lanes
- **Innovation**: "Band-first-then-lane" paradigm shift
- **Outcome**: **30% accuracy improvement**, production deployment
- **Evidence**: Full source code, build system, documentation

### [Phase 2: Algorithm Optimization](/phase2_algorithm_optimization)  
**January-February 2023** | **Methodology & Performance Analysis**
- **Problem**: Low-contrast bands, rigid output format
- **Solutions**: Global contrast enhancement, contour-aware detection
- **Outcome**: **Additional 10% accuracy gain** on challenging cases
- **Evidence**: Quantitative metrics, technical documentation

### [Phase 3: Deep Learning Exploration](/phase3_deep_learning_pilot)
**August 2023** | **Experimental Research & Comparison**
- **Research Question**: Can DL match hand-crafted algorithm performance?
- **Approach**: Faster R-CNN with domain-specific adaptations
- **Finding**: **Accuracy parity with 50-70% GPU speed advantage**
- **Evidence**: Comparative analysis, experimental results

## Technical Highlights

### Core Innovations
1. **Paradigm Shift**: "Band-first-then-lane" vs traditional "lane-first-then-band"
2. **Adaptive Processing**: Dynamic thresholding based on image characteristics
3. **Multi-Approach Validation**: Traditional CV and deep learning comparison

### Performance Impact
- **Overall**: 30% accuracy improvement over legacy systems
- **Edge Cases**: Resolved complete failure on skewed lanes
- **Production**: Deployed in biotech company's analysis software

## Repository Structure

BioLight/
├── phase1_band_first_algorithm/ # Complete C++ implementation
│ ├── src/ # Source code (PlanA/B/C versions)
│ ├── CMakeLists.txt # Build configuration
│ └── README.md # Technical documentation
├── phase2_algorithm_optimization/ # Optimization methodology
│ └── README.md # Technical solutions & results
├── phase3_deep_learning_pilot/ # ML exploration
│ └── README.md # Experimental design & findings
└── research_synthesis/ # Overall research narrative
└── README.md # Synthesis & academic relevance

## Quick Start

### Explore Phase 1 (Code Implementation)
```bash
# Clone repository
git clone https://github.com/MichaelYu0530/BioLight.git
cd BioLight/phase1_band_first_algorithm

# Build and run demonstration
mkdir build && cd build
cmake .. && make
./demo
```

## Research Significance

This work demonstrates:

- **Problem-solving**: Identifying fundamental flaws in existing approaches

- **Technical evolution**: From concept to production to research exploration

- **Methodological rigor**: Quantitative validation and comparative analysis

- **Interdisciplinary synthesis**: Statistics, computer vision, machine learning

- ## Author

  **Yue Yu**
  University of Science and Technology of China (USTC)
  Bachelor of Science in Statistics, Expected 2025

  **Contact**: yueyu0530@gmail.com
  **GitHub**: [MichaelYu0530](https://github.com/MichaelYu0530)

  ## License

  This repository is licensed under the MIT License - see the [LICENSE](https://license/) file for details.

  ## Acknowledgments

  - Mentors at **Guangzhou Biolight Biotechnology** for guidance and domain expertise
  - **USTC faculty** for statistical and mathematical foundation
  - **OpenCV and TensorFlow communities** for excellent open-source libraries