.. zephyr:code-sample:: i2s-record-playback
   :name: I2S record/playback
   :relevant-api: i2s_interface

   Record a few seconds of audio, then play it back, in a loop.

Overview
********

This sample demonstrates how to use an I2S driver together with an audio
codec to record and play back audio, on hardware that cannot run the RX and
TX streams of an I2S peripheral at the same time (e.g. most STM32 SoCs).
Instead of a live echo effect, this sample alternates between two phases:

#. Record :kconfig:option:`CONFIG_RECORD_SECONDS` seconds of audio into a RAM
   buffer.
#. Play that buffer back through the same I2S peripheral.

This cycle repeats forever. Since the whole recording is kept in RAM, the
sample's memory usage grows with the sample rate and the recording duration
(:kconfig:option:`CONFIG_SAMPLE_FREQ` and
:kconfig:option:`CONFIG_RECORD_SECONDS`); lower these values on RAM
constrained targets.

Requirements
************

The sample expects a devicetree node labelled ``audio_codec`` implementing the
audio codec API, and either a node labelled ``i2s_rxtx`` if a single I2S
peripheral is shared between the RX and TX streams, or separate ``i2s_rx``
and ``i2s_tx`` node labels otherwise.

This sample has been tested on :zephyr:board:`stm32h7s78_dk`
(stm32h7s78_dk/stm32h7s7xx/ext_flash_app), using its on-board WM8904 audio
codec.

Building and Running
*********************

The code can be found in :zephyr_file:`samples/drivers/i2s/record_playback`.

To build and flash the application:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/i2s/record_playback
   :board: stm32h7s78_dk/stm32h7s7xx/ext_flash_app
   :goals: build flash
   :compact:

Connect a microphone (or line-in source) and headphones (or a speaker) to
the audio codec, reset the board, and speak or play a sound during the
"Recording" phase: it will be played back during the following "Replaying"
phase.
