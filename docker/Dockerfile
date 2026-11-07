FROM ros:humble

ENV PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONDONTWRITEBYTECODE=1

WORKDIR /ws

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pip \
      python3-rosdep \
      libgl1 \
      libglib2.0-0 \
      libsm6 \
      libxext6 \
      libxrender1 \
      libxcb-xinerama0 \
      libxcb-icccm4 \
      libxcb-image0 \
      libxcb-keysyms1 \
      libxcb-randr0 \
      libxcb-render-util0 \
      libxcb-shape0 \
      libxcb-cursor0 \
      libxcb-xkb1 \
      libxkbcommon-x11-0 \
      xauth \
      xvfb \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install \
      --index-url https://download.pytorch.org/whl/cpu \
      torch==2.11.0 torchvision==0.26.0 \
    && python3 -m pip install ultralytics==8.4.194 --no-deps \
    && python3 -m pip install \
      'numpy>=1.23,<2' \
      'matplotlib>=3.3.0' \
      'pillow>=7.1.2' \
      'pyyaml>=5.3.1' \
      'requests>=2.23.0' \
      'scipy>=1.4.1' \
      'polars>=0.20.0' \
      'nvidia-ml-py>=12.0.0' \
      'ultralytics-thop>=2.2.3' \
      'opencv-python>=4.7.0,<4.12' \
      pytest
