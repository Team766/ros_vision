FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

RUN apt-get update && apt-get install -y sudo git vim lsb-release software-properties-common \
 && ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata \
 && dpkg-reconfigure --frontend noninteractive tzdata

COPY install_deps.sh /install_deps.sh
RUN chmod +x /install_deps.sh
RUN /install_deps.sh
ENV PATH="$PATH:/usr/local/cuda/bin"


