
# RDNT

RDNT is a project that takes an input audio source and runs signal processing algorithms to drive wired LED strips to create a synchronized audio-visual experience. These audio sources are an onboard analog microphone, an AUX-in port, and a Bluetooth signal from our associated Android app. The user will be able to toggle through the 3 modes, with the microphone being the default, by pressing an external button. This mode toggling will be achieved through a multiplexxed circuit that selects the current mode.
The signal processing algorithms that will be used are the FFT algorithm and a 1-D Gaussian filter. The Gaussian filter reduces noise on the input sample and the FFT algorithm uses this noise reduced signal to isolate frequencies. The LED strips will be driven by a message sender that accepts the output of FFT to create patterns on the wired LED strips.
To create patterns we will send PWM signals to the data pin of the LED strips, these patterns and their associated color ranges can be selected by the user using the associated Android app. The microcontroller will read input samples and compute signal processing algorithms to drive the message sender. The LED strips will be powered by a wall adapter from 120V AC to 5V DC. A switching regulator circuit will drop the 5V DC to 3.3V DC for our microcontroller. For demonstration purposes, we will construct an acrylic display.

See our website for more information: https://engineering.purdue.edu/477grp18/

Current Stage: Soldering components onto our PCB, working on final software, and creating device packaging