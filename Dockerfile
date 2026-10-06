ARG ROS_BASE_IMAGE=osrf/ros:noetic-desktop-full
FROM ${ROS_BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG CATKIN_JOBS=2
ARG USERNAME=ocs2
ARG USER_UID=1000
ARG USER_GID=1000

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        dbus-x11 \
        git \
        gnome-terminal \
        libgmp-dev \
        libmpfr-dev \
        liboctomap-dev \
        libyaml-cpp-dev \
        mesa-utils \
        python3-catkin-tools \
        ros-noetic-octomap-msgs \
        ros-noetic-pinocchio \
        xauth \
    && rm -rf /var/lib/apt/lists/*

RUN if ! getent group "${USER_GID}" >/dev/null; then groupadd --gid "${USER_GID}" "${USERNAME}"; fi \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}"

WORKDIR /workspace
COPY --chown=${USER_UID}:${USER_GID} src/ src/
COPY --chown=${USER_UID}:${USER_GID} LICENSE THIRD_PARTY_NOTICES.md CITATION.cff ./
COPY --chown=${USER_UID}:${USER_GID} LICENSES/ LICENSES/
COPY --chmod=0755 docker/entrypoint.sh /usr/local/bin/ocs2-entrypoint
RUN chown "${USER_UID}:${USER_GID}" /workspace \
    && sed -i 's/\r$//' /usr/local/bin/ocs2-entrypoint

USER ${USERNAME}

RUN source /opt/ros/noetic/setup.bash \
    && catkin init \
    && catkin config \
        --extend /opt/ros/noetic \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build ocs2_multi_robot \
        --jobs "${CATKIN_JOBS}" \
        --parallel-packages 1 \
        --no-status

ENTRYPOINT ["/usr/local/bin/ocs2-entrypoint"]
CMD ["bash"]
