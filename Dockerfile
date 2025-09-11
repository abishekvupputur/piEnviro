FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential python3-dev i2c-tools \
    libatlas-base-dev libjpeg62-turbo libopenjp2-7 libtiff6 zlib1g \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/enviroplus

# Copy everything from your repo (where Dockerfile lives) into /opt/enviroplus
COPY . .

# Install from local source
RUN python -m pip install --prefer-binary .

WORKDIR /opt/enviroplus/examples

CMD ["bash", "-lc", "python \"${EXAMPLE:-all-in-one.py}\""]

