FROM python:3.10-slim

WORKDIR /app

# System deps: Xvfb (virtual display), x11vnc, noVNC, PyBullet libs
RUN apt-get update && apt-get install -y \
    build-essential \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libx11-6 \
    libglu1-mesa \
    freeglut3-dev \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

COPY start_vnc.sh /start_vnc.sh
RUN chmod +x /start_vnc.sh

CMD ["python", "main.py"]
