#!/bin/bash
./build/RISCV/gem5.opt configs/example/se.py\
  -c disk/mnt/root/redis-6.0.10-riscv/src/redis-server \
  -o "disk/mnt/root/redis-6.0.10-riscv/redis.conf --bind 127.0.0.1" \
  --redirects /lib=disk/mnt/lib \
  --redirects /lib64=disk/mnt/lib64 \
  --redirects /usr/lib=disk/mnt/usr/lib \
  --redirects /usr/lib64=disk/mnt/usr/lib64 \
  --interp-dir disk/mnt \
  --cpu-type=DerivO3CPU --caches --l2cache
