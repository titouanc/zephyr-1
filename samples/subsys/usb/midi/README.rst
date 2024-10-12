.. zephyr:code-sample:: midi
   :name: USB MIDI device class sample
   :relevant-api: usbd_api usb_midi

   USB MIDI 2.0 device class sample

Overview
********

This sample demonstrates how to implement a USB MIDI 2.0 device. It can run on
any board with a USB device controller. This samples sends all MIDI1 messages
sent to the device back to the host. In addition, on boards that have a
user button, pressing and releasing that button will send MIDI1 note on and
note off messages.

The application exposes a single USB-MIDI interface with a single bidirectional
group terminal. This allows exchanging data with the host on a "virtual wire"
that carries MIDI1 messages, pretty much like a standard USB-MIDI in/out adapter
would provide. The loopback acts as if a real MIDI cable was connected between
the output and the input, and the button acts as if there was a user keyboard
to press a key.

Building and Running
********************

The code can be found in :zephyr_file:`samples/subsys/usb/midi`.

To build and flash the application:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/usb/midi
   :board: nucleo_f429zi
   :goals: build flash
   :compact:

Using the MIDI interface
************************

Once this sample is flashed, connect the device USB port to a host computer
with MIDI support. For example, on Linux, you can use alsa to access the device:

.. code-block:: console

  $ amidi -l
  Dir Device    Name
  IO  hw:2,1,0  Group 1 (USBD MIDI Sample)


The "USBD MIDI Sample" interface should also appear in any program with MIDI
support (for example your favorite Digital Audio Workstation)

Testing loopback
****************

Open a first shell, and start dumping MIDI events:

.. code-block:: console

  $ amidi -p hw:2,1,0 -d


Then, in a second shell, send some MIDI events (for example note-on/note-off):

.. code-block:: console

  $ amidi -p hw:2,1,0 -S "90427f 804200"

These events should then appear in the first shell (dump)

On devboards with a user button, press it and observe that there are some note
on/note off events delivered to the first shell (dump)

.. code-block:: console

  $ amidi -p hw:2,1,0 -d

  90 40 7F
  80 40 7F
  90 40 7F
  80 40 7F
  [...]
