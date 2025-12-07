---
sidebar_position: 5
---

# Hardware Requirements

## Overview

This course involves computationally intensive tasks including physics simulation, real-time perception, and AI model inference. This page outlines the hardware you'll need to successfully complete the course.

## Minimum vs. Recommended Specifications

### Workstation Requirements

| Component | Minimum | Recommended | Ideal |
|-----------|---------|-------------|-------|
| **CPU** | Intel i5-12400 / Ryzen 5 5600X | Intel i7-13700K / Ryzen 9 7900X | Intel i9-13900K / Ryzen 9 7950X |
| **GPU** | NVIDIA RTX 3060 (12GB) | NVIDIA RTX 4070 (12GB) | NVIDIA RTX 4090 (24GB) |
| **RAM** | 32GB DDR4 | 64GB DDR4/DDR5 | 128GB DDR5 |
| **Storage** | 512GB NVMe SSD | 1TB NVMe SSD + 2TB HDD | 2TB NVMe SSD + 4TB HDD |
| **OS** | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |

### Why These Requirements?

#### **GPU: NVIDIA RTX Series**
- **CUDA Cores**: Required for Isaac Sim, AI inference, and simulation
- **Tensor Cores**: Accelerate neural network training and inference
- **RT Cores**: Ray tracing for photorealistic rendering in Isaac Sim
- **VRAM**: 12GB minimum for complex scenes; 24GB for large-scale simulations

**Note**: AMD GPUs are not currently supported by NVIDIA Isaac ecosystem.

#### **CPU: Multi-Core Performance**
- Physics simulation (Gazebo, Isaac Sim) is CPU-intensive
- ROS 2 runs multiple concurrent nodes
- Compiling large codebases (ROS packages) benefits from many cores

#### **RAM: 64GB Sweet Spot**
- Isaac Sim alone can use 16-32GB
- Multiple Docker containers for different environments
- Browser, IDE, documentation simultaneously open
- 32GB is bare minimum; 64GB recommended; 128GB for heavy workflows

#### **Storage: Fast NVMe SSD**
- Ubuntu OS, ROS 2, Isaac Sim, Unity: ~200GB combined
- Docker images and containers: 50-100GB
- Datasets (robot meshes, training data): 100-500GB
- Fast read/write speeds reduce load times significantly

#### **Operating System: Ubuntu 22.04 LTS**
- ROS 2 Humble officially supports Ubuntu 22.04
- NVIDIA drivers and CUDA work best on Ubuntu
- Most robotics tools are Linux-first
- **Windows/WSL2**: Possible but not recommended for this course
- **macOS**: Not supported for NVIDIA tools

## Software Requirements

You'll install the following major software packages:

### Core Development Environment
- **Ubuntu 22.04 LTS** (Jammy Jellyfish)
- **ROS 2 Humble** (LTS release)
- **Python 3.10+**
- **Git** for version control

### Simulation Platforms
- **Gazebo Fortress** or **Gazebo Classic 11**
- **NVIDIA Isaac Sim 2023.1+** (requires RTX GPU)
- **Unity 2022 LTS** (optional, for advanced visualization)

### AI/ML Tools
- **NVIDIA CUDA Toolkit 11.8+**
- **cuDNN** (CUDA Deep Neural Network library)
- **PyTorch** or **TensorFlow** (for AI models)
- **OpenAI API** (for LLM integration in Chapter 4)

### Development Tools
- **Visual Studio Code** or **CLion** (IDE)
- **Docker** (for containerized environments)
- **Jupyter Notebooks** (for interactive development)

## Optional: Edge AI Kit

For students who want hands-on embedded AI experience:

### NVIDIA Jetson (Optional)

| Model | Performance | Price | Use Case |
|-------|-------------|-------|----------|
| **Jetson Orin Nano** | Entry-level AI | ~$500 | Learning, prototyping |
| **Jetson Orin NX** | Mid-range AI | ~$1000 | Moderate workloads |
| **Jetson AGX Orin** | High-end AI | ~$2000 | Production-ready systems |

**Benefits**:
- Deploy models to real embedded hardware
- Learn about power/performance tradeoffs
- Experience real-time constraints

**Not Required**: All course work can be completed in simulation on your workstation.

## Optional: Physical Robot Platforms

For the ultimate hands-on experience (not required):

### Unitree Robotics

#### **Unitree Go2**
- **Type**: Quadruped robot dog
- **Price**: $1,600 - $3,000 (depending on variant)
- **Capabilities**: Walking, running, obstacle avoidance
- **Programming**: ROS 2 support, Python SDK
- **Use Case**: Learn locomotion and navigation

#### **Unitree G1**
- **Type**: Humanoid robot
- **Price**: ~$16,000+
- **Capabilities**: Bipedal walking, manipulation
- **Use Case**: Full humanoid development platform

**Note**: These are optional enrichment. The course is entirely completable in simulation.

## Cloud/Remote Options

If you don't have the required hardware:

### Cloud GPU Instances

- **AWS EC2 (g5.xlarge, g5.2xlarge)**: NVIDIA A10G GPUs
- **Google Cloud (n1-standard + T4/V100)**: Cost-effective GPU compute
- **Paperspace Gradient**: Ready-made ML environments
- **Lambda Labs**: Dedicated GPU instances

**Estimated Cost**: $1-3/hour for suitable instances

**Pros**: No upfront hardware cost  
**Cons**: Ongoing rental fees, latency, complex GUI forwarding

### University Lab Workstations

Check if your institution provides access to GPU workstations or compute clusters.

## Network Requirements

- **Bandwidth**: 50+ Mbps (for downloading large assets, Docker images)
- **Latency**: Low-latency connection if using cloud instances
- **Storage**: Consider cloud backup for datasets and projects

## Budget Guidance

### Conservative Budget (~$1,500-2,000)
- Used/refurbished workstation with RTX 3060
- 32GB RAM, upgrade to 64GB later
- 512GB SSD
- Use cloud instances when necessary

### Recommended Budget (~$3,000-4,000)
- New build: Ryzen 9 7900X + RTX 4070
- 64GB DDR5 RAM
- 1TB NVMe SSD
- Covers entire course comfortably

### Premium Setup (~$5,000-7,000)
- High-end build: i9-13900K + RTX 4090
- 128GB DDR5 RAM
- 2TB NVMe + 4TB HDD
- Future-proof for research/professional work

## Pre-Course Checklist

Before the course begins, ensure you have:

- [ ] Ubuntu 22.04 LTS installed (dual-boot or dedicated machine)
- [ ] NVIDIA drivers installed and verified
- [ ] At least 500GB free disk space
- [ ] Stable internet connection
- [ ] GitHub account created
- [ ] Familiarity with Linux terminal basics

## Troubleshooting Common Issues

### "My GPU isn't recognized"
- Install proprietary NVIDIA drivers: `sudo ubuntu-drivers autoinstall`
- Verify installation: `nvidia-smi`

### "I only have Windows"
- **Option 1**: Dual-boot Ubuntu (recommended)
- **Option 2**: WSL2 + Docker (limited Isaac Sim support)
- **Option 3**: Cloud GPU instance

### "I don't have an NVIDIA GPU"
- Some parts of the course (ROS 2, Gazebo) will work
- Isaac Sim requires NVIDIA; consider cloud instances for those modules

## Questions?

If you have hardware questions or need advice on building/buying a system for this course, please reach out to the course staff or community forums.

---

✅ **Chapter 1 Complete!**

You now understand the foundations of Physical AI, the robotics landscape, sensor systems, and hardware requirements.

👉 **[Next: Chapter 2 - ROS 2 Foundations →](../chapter2)**

Or return to: **[Chapter 1 Overview](./index)**
