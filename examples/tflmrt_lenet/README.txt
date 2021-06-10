# examples/tflmrt_lenet

This is a simple example of the LeNet-5's inference using `sdk/modules/tflmrt`.
This example loads an hand-written digit image and a neural network model
and does forward propagation after feeding the image.
Then, it outputs probabilities of digits[0-9] as classification result.

## Configuration Pre-requisites:

This application depends on `tflmrt` configuration:

* CONFIG_TFLM_RT     - tflmrt

## Example Configuration:

* CONFIG_EXAMPLES_TFLMRT_LENET                    - Enable this example
* CONFIG_EXAMPLES_TFLMRT_LENET_PROGNAME           - Program name
* CONFIG_EXAMPLES_TFLMRT_LENET_PRIORITY           - Example priority (default: 100)
* CONFIG_EXAMPLES_TFLMRT_LENET_STACKSIZE          - Example stack size (default: 2048)

## Operation:

Run `tflmrt_lenet`. By default, the application uses the built-in model and image.

### command usage:

Install `nuttx.spk` into CXD5602. You can now execute `tflmrt_lenet` on
the target-side console. This command usage is as below:

```
SYNOPSIS
       tflmrt_lenet [-s] [tflite] [pgm]

DESCRIPTION
       tflmrt_lenet instantiates a neural network defined by
       tflite model (default: built-in TensorFlow Lite model),
       and feeds an image (default: built-in test image).

OPTIONS
       -s     : skip image normalization before feeding into the network.
                if no -s option is given, image data is divided by 255.0

       tflite : path to TF Lite model, if not given or "default", then use
                built-in TensorFlow Lite model.

       pgm    : path to pgm image, if not given use built-in test image (0)
```

### expected output:

`tflmrt_lenet` prints a 1D-array which TensorFlow Lite model outputs as `output[0-9]`.
For example, execute `tflmrt_lenet` like below. Then, you can refer to `output[0-9]`
as probabilities that each digit is drawn. In this example, since you feed built-in
test image (0), the corresponding `output[0]` should be almost 1.0.

```
nsh> tflmrt_lenet
Load built-in model
Load built-in test image
Image Normalization (1.0/255.0): enabled
start tflm_runtime_forward()
output[0]=0.996093
output[1]=0.000000
output[2]=0.000000
output[3]=0.000000
output[4]=0.000000
output[5]=0.000000
output[6]=0.000000
output[7]=0.000000
output[8]=0.000000
output[9]=0.000000
...
```
