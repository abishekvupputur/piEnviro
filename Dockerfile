FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential python3-dev i2c-tools \
    libjpeg62-turbo libopenjp2-7 zlib1g \
    libasound2 libportaudio2\
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/enviroplus

# Copy everything from your repo (where Dockerfile lives) into /opt/enviroplus
COPY . .

# Install from local source
RUN python -m pip install --prefer-binary .

RUN if [ -f requirements-examples.txt ]; then \
      python -m pip install --prefer-binary -r requirements-examples.txt; \
    fi
RUN python -m pip install --prefer-binary sounddevice


WORKDIR /opt/enviroplus/examples

CMD ["python3", "enviroMultiClient.py"]
