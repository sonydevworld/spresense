=== Overview of this digital PLL worker ===

  This PLL will lock the phase based on GNSS 1-PPS signal.
  There are 3 steps to lock.

  1. Measure period of 1-PPS signal
      1-PPS signal is much correctly signal for measure just 1 seconds.
      Spresense audio clock ossilator frequency has dispersion.
      To avoid it, 1-PPS signal period is measured at first, and the value
      is used as sampling rate.
  2. Phase shift of reference signal to fit the edge of 1-PPS
      This worker generates a reference signal in every 1 second.
      This signal is loopbacked from speaker output to mic input.
      The signal's frequency is 200Hz as default, and generated just one
      cycle in the 1 secound period.
      To fit a negative zero-cross point to the edge of 1-PPS, the period
      from 1-PPS edge to the zero-cross point is measured and adjust phase
      counter of it.
  3. Adjust details (This steop is already locked)
      To adjust subtle misalignment of the zero-cross and the edge,
      reference signal value value on the edge is checked if it is zero or
      not. Because the difference from zero is the phase difference,
      Then adjust the phase using it. This action is processed in every
      edges.
  After locked, carrier signal is generated and modulate PSK for sending
  128bit data.

  State transition
  ~~~~~~~~~~~~~~~~

  To achieve to the steps, This module has 7 stages to lock the phase.
  State transition is as below:
     +-------+
     | READY |
     +-------+
         | Receive PLL_START msg from alusr_pll class
         V
  +---------------+
  | 1STEDGEDETECT |
  +---------------+
         | Detect the rising edge of 1PPS
         |  and start counting
         V
  +---------------+
  | 2NDEDGEDETECT |
  +---------------+
         | Detect the rising edge of 1PPS
         |  and stop counting and update 1hz period (actual sampling rate)
         V
    +----------+
    | INTERVAL | Changing sampling freq refrects generating signals
    |          | to stabilizing signals, wait one pps as interval
    +----------+
         | Wait "EDGEDETECT_INTERVAL_SEC" times detection of
         | the rising edge of 1PPS and check the zerocross point
         |
         +------------+
         |            | Case of not detected
         |            |  start counting the period from the edge to
         |            |  zero-cross point of reference siganl.
         |            V
         |      +------------+ Found zero-cross point and update a phase
         |      | PHASESHIFT |--------------------------+
         |      +------------+                          |
         |                                              |
         | Case of detected                             V
         |  update a phase of reference signal    +--------------+
         +--------------------------------------> | PHASEUPDATED | 
                                                  +--------------+
                         Detect the rising edge of 1PPS |
                       +--------------------------------+
                       |
                       V
                +-------------+
          +---> | PHASELOCKED |
          |     +-------------+
          |            | Detect the rising edge of 1PPS
          +------------+  and adjust phase of reference signal.
  
  It will move to READY state if PLL_STOP message is received in all state.

