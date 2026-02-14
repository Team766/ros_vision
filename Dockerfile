FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

RUN apt-get update && apt-get install -y sudo git vim lsb-release software-properties-common \
 && ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && rm -rf /usr/share/gradle* \
 && rm -rf /usr/share/apache-maven* \
 && rm -rf /usr/share/sbt \
 && rm -rf /usr/local/aws-* \
 && rm -rf /usr/local/share/vcpkg \
 && rm -rf /imagegeneration \
 && rm -rf /opt/pipx \
 && rm -rf /usr/lib/jvm \
 && rm -rf /usr/lib/google-cloud-sdk \
 && rm -rf /usr/lib/mono \
 && rm -rf /usr/lib/heroku \
 && rm -rf /usr/share/dotnet \
 && rm -rf /usr/local/lib/android \
 && rm -rf /opt/ghc \
 && rm -rf /opt/hostedtoolcache \
 && rm -rf /usr/local/graalvm/ \
 && rm -rf /usr/local/.ghcup/ \
 && rm -rf /usr/local/share/powershell \
 && rm -rf /usr/local/share/chromium \
 && rm -rf /usr/local/lib/node_modules \
 && rm -rf /opt/microsoft \
 && rm -rf /opt/az \
 && rm -rf /opt/google \
 && rm -rf /usr/share/swift \
 && rm -rf /usr/share/miniconda \
 && rm -rf /usr/local/julia* \
 && rm -rf /usr/share/kotlinc \
 && rm -rf /usr/share/gradle* \
 && rm -rf /usr/share/apache-maven* \
 && rm -rf /usr/share/sbt \
 && rm -rf /usr/local/aws-* \
 && rm -rf /usr/local/share/vcpkg \
 && rm -rf /imagegeneration \
 && rm -rf /opt/pipx \
 && rm -rf /usr/lib/jvm \
 && rm -rf /usr/lib/google-cloud-sdk \
 && rm -rf /usr/lib/mono \
 && rm -rf /usr/lib/heroku \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/* \
 && docker system prune -af \
 && docker builder prune -af

COPY install_deps.sh /install_deps.sh
RUN chmod +x /install_deps.sh
RUN /install_deps.sh \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/* \
 && rm -rf /tmp/*
ENV PATH="$PATH:/usr/local/cuda/bin"

# Create non-root user for development
RUN useradd -m -s /bin/bash -G sudo vscode \
 && echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/vscode

USER vscode
WORKDIR /workspaces/ros_vision
