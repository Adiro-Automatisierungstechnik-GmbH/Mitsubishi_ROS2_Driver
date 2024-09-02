FROM debian:buster-slim

# Install git, supervisor, VNC, & X11 packages
RUN apt-get update && apt-get install -y \
    bash \
    ca-certificates \
    fluxbox \
    git \
    net-tools \
    novnc \
    procps \
    python3 \
    socat \
    supervisor \
    x11vnc \
    xterm \
    xvfb \
    # Clear apt-cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

# Setup demo environment variables
ENV HOME=/root \
    DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:0.0 \
    DISPLAY_WIDTH=1920 \
    DISPLAY_HEIGHT=1080 \
    RUN_FLUXBOX=yes     

COPY ./ /app
COPY ./index.html /usr/share/novnc/
RUN chmod +x /app/entrypoint.sh
# modify file to LF line endings
RUN sed -i 's/\r$//' /app/entrypoint.sh

CMD ["/app/entrypoint.sh"]
EXPOSE 8080
