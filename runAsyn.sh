#! /bin/bash

app=$1
workdir=$3
testOption=$2
debugs=$4

if [[ $debugs != "" ]]; then
    if [[ $testOption == "hop0" ]]; then
        build/RISCV/gem5.opt --debug-flags=$debugs -d m5outAsync/0Level/$workdir configs/example/se.py \
        -c $app,$app -o 209715200,209715200 --redirects /lib=disk/mnt3/lib  \
        --redirects /lib64=disk/mnt3/lib64   --redirects /usr/lib=disk/mnt3/usr/lib  \
        --redirects /usr/lib64=disk/mnt3/usr/lib64   --interp-dir disk/mnt3  \
        --cpu-type=DerivO3CPU --caches --l2cache --mem-type=DDR4_2400_8x8 \
        --l1d_size=16kB --l1d_assoc=2 --l2_size=256kB --l2_assoc=8 --num-cpus=2 \
        --async-memory --l1-enable-bank --l2-enable-bank --disagregate_mem \
        --l1-num-bank=4 --l2-num-bank=2 --cpu-clock=2GHz
    elif [[ $testOption == "hop1" ]]; then
        build/RISCV/gem5.opt --debug-flags=$debugs -d m5outAsync/1Level/$workdir configs/example/se.py \
        -c $app,$app -o 209715200,209715200 --redirects /lib=disk/mnt3/lib  \
        --redirects /lib64=disk/mnt3/lib64   --redirects /usr/lib=disk/mnt3/usr/lib  \
        --redirects /usr/lib64=disk/mnt3/usr/lib64   --interp-dir disk/mnt3  \
        --cpu-type=DerivO3CPU --caches --l2cache --mem-type=DDR4_2400_8x8 \
        --l1d_size=16kB --l1d_assoc=2 --l2_size=256kB --l2_assoc=8 --num-cpus=2 \
        --async-memory --l1-enable-bank --l2-enable-bank --disagregate_mem \
        --mem-size=1MB --test-L1Remote \
        --l1-num-bank=4 --l2-num-bank=2 --cpu-clock=2GHz
    elif [[ $testOption == "hop2" ]]; then
        build/RISCV/gem5.opt --debug-flags=$debugs -d m5outAsync/2Level/$workdir configs/example/se.py \
        -c $app,$app -o 209715200,209715200 --redirects /lib=disk/mnt3/lib  \
        --redirects /lib64=disk/mnt3/lib64   --redirects /usr/lib=disk/mnt3/usr/lib  \
        --redirects /usr/lib64=disk/mnt3/usr/lib64   --interp-dir disk/mnt3  \
        --cpu-type=DerivO3CPU --caches --l2cache --mem-type=DDR4_2400_8x8 \
        --l1d_size=16kB --l1d_assoc=2 --l2_size=256kB --l2_assoc=8 --num-cpus=2 \
        --async-memory --l1-enable-bank --l2-enable-bank --disagregate_mem \
        --mem-size=1MB \
        --l1-num-bank=4 --l2-num-bank=2 --cpu-clock=2GHz
    elif [[ $testOption == "hybrid" ]]; then
        build/RISCV/gem5.opt --debug-flags=$debugs -d m5outAsync/hybrid/$workdir configs/example/se.py \
        -c $app,$app -o 209715200,209715200 --redirects /lib=disk/mnt3/lib  \
        --redirects /lib64=disk/mnt3/lib64   --redirects /usr/lib=disk/mnt3/usr/lib  \
        --redirects /usr/lib64=disk/mnt3/usr/lib64   --interp-dir disk/mnt3  \
        --cpu-type=DerivO3CPU --caches --l2cache --mem-type=DDR4_2400_8x8 \
        --l1d_size=16kB --l1d_assoc=2 --l2_size=256kB --l2_assoc=8 --num-cpus=2 \
        --async-memory --l1-enable-bank --l2-enable-bank --disagregate_mem \
        --mem-size=1MB \
        --test-hybrid \
        --l1-num-bank=4 --l2-num-bank=2 --cpu-clock=2GHz
    fi
else
    if [[ $testOption == "hop0" ]]; then
        build/RISCV/gem5.opt -d m5outAsync/0Level/$workdir configs/example/se.py \
        -c $app,$app -o 209715200,209715200 --redirects /lib=disk/mnt3/lib  \
        --redirects /lib64=disk/mnt3/lib64   --redirects /usr/lib=disk/mnt3/usr/lib  \
        --redirects /usr/lib64=disk/mnt3/usr/lib64   --interp-dir disk/mnt3  \
        --cpu-type=DerivO3CPU --caches --l2cache --mem-type=DDR4_2400_8x8 \
        --l1d_size=16kB --l1d_assoc=2 --l2_size=256kB --l2_assoc=8 --num-cpus=2 \
        --async-memory --l1-enable-bank --l2-enable-bank --disagregate_mem \
        --l1-num-bank=4 --l2-num-bank=2 --cpu-clock=2GHz
    elif [[ $testOption == "hop1" ]]; then
        build/RISCV/gem5.opt -d m5outAsync/1Level/$workdir configs/example/se.py \
        -c $app,$app -o 209715200,209715200 --redirects /lib=disk/mnt3/lib  \
        --redirects /lib64=disk/mnt3/lib64   --redirects /usr/lib=disk/mnt3/usr/lib  \
        --redirects /usr/lib64=disk/mnt3/usr/lib64   --interp-dir disk/mnt3  \
        --cpu-type=DerivO3CPU --caches --l2cache --mem-type=DDR4_2400_8x8 \
        --l1d_size=16kB --l1d_assoc=2 --l2_size=256kB --l2_assoc=8 --num-cpus=2 \
        --async-memory --l1-enable-bank --l2-enable-bank --disagregate_mem \
        --mem-size=1MB --test-L1Remote \
        --l1-num-bank=4 --l2-num-bank=2 --cpu-clock=2GHz
    elif [[ $testOption == "hop2" ]]; then
        build/RISCV/gem5.opt -d m5outAsync/2Level/$workdir configs/example/se.py \
        -c $app,$app -o 209715200,209715200 --redirects /lib=disk/mnt3/lib  \
        --redirects /lib64=disk/mnt3/lib64   --redirects /usr/lib=disk/mnt3/usr/lib  \
        --redirects /usr/lib64=disk/mnt3/usr/lib64   --interp-dir disk/mnt3  \
        --cpu-type=DerivO3CPU --caches --l2cache --mem-type=DDR4_2400_8x8 \
        --l1d_size=16kB --l1d_assoc=2 --l2_size=256kB --l2_assoc=8 --num-cpus=2 \
        --async-memory --l1-enable-bank --l2-enable-bank --disagregate_mem \
        --mem-size=1MB \
        --l1-num-bank=4 --l2-num-bank=2 --cpu-clock=2GHz
    elif [[ $testOption == "hybrid" ]]; then
        build/RISCV/gem5.opt -d m5outAsync/hybrid/$workdir configs/example/se.py \
        -c $app,$app -o 209715200,209715200 --redirects /lib=disk/mnt3/lib  \
        --redirects /lib64=disk/mnt3/lib64   --redirects /usr/lib=disk/mnt3/usr/lib  \
        --redirects /usr/lib64=disk/mnt3/usr/lib64   --interp-dir disk/mnt3  \
        --cpu-type=DerivO3CPU --caches --l2cache --mem-type=DDR4_2400_8x8 \
        --l1d_size=16kB --l1d_assoc=2 --l2_size=256kB --l2_assoc=8 --num-cpus=2 \
        --async-memory --l1-enable-bank --l2-enable-bank --disagregate_mem \
        --mem-size=1MB \
        --test-hybrid \
        --l1-num-bank=4 --l2-num-bank=2 --cpu-clock=2GHz
    fi
fi