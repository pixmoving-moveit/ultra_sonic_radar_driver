#!/bin/bash

probe1=1000
probe2=2000
probe3=3000
probe4=4000

while true; do
  cansend can1 611#$probe1$probe2$probe3$probe4
  cansend can1 612#$probe1$probe2$probe3$probe4
  cansend can1 613#$probe1$probe2$probe3$probe4

  sleep 0.05
done
