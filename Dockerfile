FROM python:3.13-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
    g++ cmake make libgomp1 ffmpeg libegl1 libgl1 libgl1-mesa-dri \
    libcairo2 libglib2.0-0 libxrender1 libsm6 libxext6 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /work
COPY . /work
RUN python -m pip install --no-cache-dir -e '.[dev]'
ENV MECHANISM_LAB_ROOT=/work PYTHONUNBUFFERED=1 LP_NUM_THREADS=4 OMP_NUM_THREADS=4
ENTRYPOINT ["lab"]
CMD ["doctor"]
