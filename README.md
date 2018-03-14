# Hardware Interview Questions

## Introduction

This repository contains a collection of commonly asked VLSI/RTL
interview questions. Solutions are presented as fully synthesizable
System Verilog along with a fully self-checking SystemC/SCV
verification environment.

Important reminder: these are not necessarily questions I myself ask,
or have been asked in the past and any references of companies is
purely coincidental.

## System Requirements
* cmake >= 3.2
* systemc >= 2.3.1
* scv >= 2.0
* verilator >= 3.9

## Build Steps (SIM)
~~~~
git clone https://github.com/stephenry/hw_interview_questions
cd hw_interview_questions
git submodule update --init --recursive
mkdir build
cd build
cmake ../
make -j
~~~~

## Test Steps
From the build directory, a full regression can be run using:
~~~~
ctest .
~~~~

## PD (VIVADO)

A standard Vivado flow is supported for each answer. PD libaries must
be explicitly selected during configuration (below). Within each answer,
a new target 'vivado' is present that invokes a standard Vivado flow.

~~~~
cmake ../ -DTARGET_VIVADO
make vivado
~~~~

## Discussion

The solutions presents herein are commonly seen questions during
hardware/RTL interviews. They are generally poised to test a
candidates knowledge of design, logic and RTL. Admittedly, some
questions are somewhat arbitrary rely upon lesser known tricks of the
trade that, although useful in the context of an intervew, are perhaps
irrelevant for day-to-day roles.

## Problems

### Design

* sorted_lists
* vending_machine_fsm
* vending_machine_dp

### General Logic

* one_or_two
* multi_counter
* multi_counter_variants
* count_ones
* detect_sequence
* missing_duplicated_word
* zero_indices_slow
* zero_indices_fast
* count_zeros_32
* fibonacci

### Arithmetic

* increment
* multiply_by_21
* ultra_wide_accumulator
* fused_multiply_add
* simd
* div_by_3

### Clocking

* mcp_formulation
* clk_div_by_3

### General Problem Solving

* latency
* using_full_adders
* gates_from_MUX2X1

### Queues

* fifo_async
* fifo_multi_push
* fifo_n
* fifo_sr
* fifo_ptr
* linked_list_fifo
* doubly_linked_list

## Noteworthy mentions

### sorted_lists

The 'sorted_lists' solution presents an implementation of a fairly
complex design problem: A machine maintains N contexts, each context
contains an ordered list of key/value pairs. On alternating cycles,
key/value pairs can be added, removed, deleted, changed. At the same
time, any list can be queried to find the n'th largest element in the
context, at a rate of upto 1 query per cycle. Additionally, any
solution must run at around 200 MHz on an FPGA.

### linked_list_fifo

An implementation of a N-context singly, linked list FIFO.

### fifo_async

An implementation of a standard asynchronous FIFO. This in particular
is a very common interview question and, although very common in
industry, presents a number of very small, fine details that very
often go unnoted.

### SIMD

An implementation of a Single-Instruction, Multiple-Data ALU. This is
quite straight-forward conceptually, however there are a few
interesting details/tricks that can be applied to facilitate an
efficient and fast implementation in silicon.
