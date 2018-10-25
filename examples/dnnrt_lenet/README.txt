# examples/dnnrt_lenet

This is a simple example of the LeNet-5's inference using `sdk/modules/dnnrt`.  
This example loads an hand-written digit image (pgm) and a neural network model (nnb) from an SD Card  
and does forward propagation after feeding the image.  
Then, it outputs probabilities of digits[0-9] as classification result.

## Configuration Pre-requisites:

This application depends on `dnnrt` and `sdcard` configuration:

* CONFIG_DNN_RT     - dnnrt
* CONFIG_CXD56_SDIO - SDIO SD Card

## Example Configuration:

* CONFIG_EXAMPLES_DNNRT_LENET           - Enable this example
* CONFIG_EXAMPLES_DNNRT_LENET_PROGNAME  - Program name
* CONFIG_EXAMPLES_DNNRT_LENET_PRIORITY  - Example priority (default: 100)
* CONFIG_EXAMPLES_DNNRT_LENET_STACKSIZE - Example stack size (default: 2048)

## Operation:

Run `dnnrt_lenet` after storing `lenet-5.nnb` and hand-written images onto SD card.

### file arrangement on the host-side:

Train the LeNet-5 model and export it as `lenet-5.nnb`
Detailed instructions for this operation are available in the following page:

http://developer.sony.com/develop/spresense/developer-tools/get-started-using-nuttx/set-up-the-nuttx-environment
(TODO: fix this URL when the corresponding chapter is created)

### file arrangement on SD card:

First, copy a directory, `examples/dnnrt_lenet/lenet-5/`, onto the root of SD card as below.  
The `lenet-5` directory holds 10 hand-written digit images, `[0-9].pgm`, under `lenet-5/data/`.  
These images were drawn by _dnnrt_ developers and their filenames are labels for each image (i.e. 1 is drawn in `1.pgm`).  
You can confirm actual images by `display` command in ImageMagick, etc.

```
$ cd Spresense.git/examples/dnnrt_lenet/
$ cp -r lenet-5 <sd mount point>
```

Next, locate `lenet-5.nnb` inside a neighboring directory, `lenet-5/model/`.

```
$ cp <somewhere>/lenet-5.nnb  <sd mount point>/lenet-5/model/
```

### command usage:

After inserting the SD card, install `nuttx.spk` into CXD5602.  
You can now execute `dnnrt_lenet` on the target-side console.  
This command usage is as below:

```
SYNOPSIS
       dnnrt_lenet [-s] [nnb] [pgm]

DESCRIPTION
       dnnrt_lenet instantiates a neural network
       defined by nnb (default value: /mnt/sd0/lenet-5/model/lenet-5.nnb),
       and feeds an image of pgm (default value: /mnt/sd0/lenet-5/data/0.pgm).

OPTIONS
       -s: skip image normalization before feeding into the network.
           if no -s option is given, image data is divided by 255.0.
```

### expected output:

`dnnrt_lenet` prints a 1D-array which `lenet-5.nnb` outputs as `output[0-9]`.  
Users are supposed to have a slight different interpretation about this `output[0-9]`   
depending on which tool -- NNC or NNabla -- was used in training.  
Expected usage of `dnnrt_lenet` differs as well like below.  

### Neural Network Console:

For example, execute `dnnrt_lenet` like below if you give `lenet-5.nnb` derived from NNC.  
Then, you can refer to `output[0-9]` as probabilities that each digit is drawn.  
In this example, since you feed `3.pgm`, the corresponding `output[3]` should be almost 1.0.  

```
nsh> dnnrt_lenet /mnt/sd0/lenet-5/model/lenet-5.nnb /mnt/sd0/lenet-5/data/3.pgm
load nnb file: /mnt/sd0/lenet-5/model/lenet-5.nnb
load pnm image: /mnt/sd0/lenet-5/data/3.pgm # 3 is hand-written
normalization: divide image data by 255.0 # normalization is done in the application-side
...
start dnn_runtime_forward()
output[0]=0.000000
output[1]=0.000000
output[2]=0.000000
output[3]=0.999976 # probability that 3 is written in the given image
output[4]=0.000000
output[5]=0.000017
output[6]=0.000000
output[7]=0.000000
output[8]=0.000006
output[9]=0.000000
...
```

### NNabla:

On the other hand, **add -s option** if you use `lenet-5.nnb` derived from NNabla.  
Otherwise, you will see that an pgm image would be misclassified.  
This is because `classification.py` embeds the normalization process in `lenet_result.nnp` as a "MulScalar" function.  
As a result, dnnrt divides image data by 255.0 inside `dnn_runtime_forward()` instead of the application-side.  
Also notice that `outputs[3]` exceeds 1.0 and some of the other elements become less than 0.  
This difference is because `classification.py` excludes a "Softmax" function,
which normalizes outputs as probabilities.  
These outputs are difficult for humans to understand at a glance,  
but you can regard an index of the largest element as a classification result as well as the above case.

```
nsh> dnnrt_lenet -s /mnt/sd0/lenet-5/model/lenet-5.nnb /mnt/sd0/lenet-5/data/3.pgm
load nnb file: /mnt/sd0/lenet-5/model/lenet-5.nnb
load pnm image: /mnt/sd0/lenet-5/data/3.pgm # 3 is hand-written
normalization: skipped # the application-side is NOT involved in normalization
...
output[0]=-21.355482
output[1]=-6.984842
output[2]=-5.412174
output[3]=20.801638 # largest, but this value might become greater than 1
output[4]=-9.799733
output[5]=7.299200
output[6]=-21.248144
output[7]=4.409724
output[8]=7.356676
output[9]=7.378049
...
```
