examples/multi_webcamera/linux_video
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  This directory contains linux driver code and user space code for
  receiving jpeg camera images from spresense multiwebcam and injecting
  them into linux video sub-system.
  This allows the Spresense camera to be treated as a normal camera device on Linux.

  This code has been confirmed on Ubuntu 22.04 LTS running on Intel(R) Core(TM) i5-5200U CPU.

  [Overview]

    This sample works like below:

       On Spresense                On Linux userspace                On Linux Kernel space
      +-------------+               +--------------+                   +---------------+
      | multiwebcam | -- Network -> | sprcam_net   | -- device file -> | spr_camera.ko | -->  To Linux Video sub-system
      +-------------+               +--------------+                   +---------------+

    'sprcam_net' gets a jpeg image from 'multiwebcam' running on Spresense via TCP/IP network.
    After got it, 'sprcam_net' inject the jpeg image into the linux kernel driver 'spr_camera.ko'.
    'spr_camera.ko' receives the jpeg image and inject it to linux video sub-system.
    Then, the image from Spresense can be used as standard video device in linux operating system.

  [How to build]

    On Spresense side : Build multiwebcam example without an option 'CONFIG_EXAMPLES_MULTIWEBCAM_USE_HTTPMJPEG' .

    On Linux PC side:
      At first, you need to prepare linux kernel driver build environment. Search web with some keyword like "linux kernel driver build environment".
      From this point forward, we will assume that the Linux driver development environment has bee established.

      Build Kernel module

        Go to "driver" directory, and execute "make" command on it.
        If it is successed, "spr_camera.ko" file is generated.

      Build User Space program

        Go to "userspace" directory, and execute "make" command on it.
        If it is successed, "sprcam_net" file is generated.

  [How to run]

    On Spresense side:
      Connect network in the same domain connected by the Linux PC.
      And check IP address on the Spresense.
      After that, run 'multiwebcam' sample app on the Spresense.

    On Linux PC side:
      In 'driver' directory, execute "make insmod" to activate built kernel module.
      Then go to 'userspace' directory, and execute "sudo ./sprcam_net <spresense IP address>".

    Then spresense's camera jpeg image is injecting into linux video sub-system.

    Open an application with using camera, then you can see and select camera device named "sprcamera".

===== 日本語 =====

  このディレクトリには、SpresenseのmultiwebcamアプリからJPEG画像を受け取り、LinuxのVideoサブシステムに注入するための
  Linuxドライバコードとユーザースペースで動作するアプリのコードが入っています。
  これを利用することで、SpresenseのカメラをLinux上の通常のカメラデバイスとして扱うことができます。

  このコードはUbuntu 22.04 LTS で動作することを確認しています。

  [概要]

  このサンプルは以下のように動作します。

       On Spresense                On Linux userspace                On Linux Kernel space
      +-------------+               +--------------+                   +---------------+
      | multiwebcam | -- Network -> | sprcam_net   | -- device file -> | spr_camera.ko | -->  To Linux Video sub-system
      +-------------+               +--------------+                   +---------------+

    Linux上でユーザー空間で動作する'sprcam_net'は、TCP/IPネットワークを介してSpresense上で動作する'multiwebcam'から、
    JPEG画像を取得します。取得したJPEG画像をKernel空間で動作するKernel Module（ドライバ）'spr_camera.ko'にデバイスファイルを介して送ります。
    'spr_camera.ko'では、受け取ったJPEG画像をLinuxのVideoサブシステムに注入し、Spresenseのカメラ画像を通常のVideoデバイスとして機能させます。

  なお、このコードは Intel(R) Core(TM) i5-5200U CPU上で動作している Ubuntu 22.04 LTS の環境で動作確認をしています。

  [ビルド方法]

    Spresense側：multiwebcameraサンプルを、Kconfigオプション 'CONFIG_EXAMPLES_MULTIWEBCAM_USE_HTTPMJPEG' を外したものでビルドします。

    Linux PC側：

      まずはじめに、Linux Kernelドライバのビルド環境を構築する必要があります。検索エンジンを用いて、 "linux kernel driver build environment" などと
      いったキーワードで検索をして、適宜環境構築をしてください。
      以下の説明では、ビルド環境が整っていることを前提に説明します。

      Kernel Moduleのビルド（spr_camera.koのビルド）

        'driver' ディレクトリに入り、"make" とコマンドを打つと、ビルドが実行され、
        正常にビルドが完了すると、同じディレクトリに 'spr_camera.ko' というファイルが生成されます。

      User Spaceのプログラムのビルド（sprcam_netのビルド）

        'userspace' ディレクトリに入り、"make" とコマンドを打つと、ビルドが実行され、
        正常にビルドが完了すると、同じディレクトリに 'sprcam_net' というファイルが生成されます。

  [実行方法]

    Spresense側：

      まずLinux PCが接続されているネットワークドメインへSpresenseを接続します。
      （例えば、Wi-Fiの場合、PCと同じWi-Fi アクセスポイントに接続します）
      その後、Spresenseの実機に振られたIPアドレスを確認し、
      'multiwebcam' サンプルを実行します。

    Linux PC側：

      まず、 'driver' ディレクトリにて、 "make insmod" と実行して、ビルドしたKernel ModuleをLinuxKernelに登録します。
      その後、 'userspace' ディレクトリに移り、 "sudo ./sprcam_net <SpresenseのIPアドレス>" と実行します。

      これでLiinux上にSpresenseのカメラデバイスが出来、そのカメラデバイスを介してSpresenseの画像を利用することが出来ます。

      試しに、カメラを使うような、Linux上の通常のアプリを実行すると、カメラデバイス選択に 'sprcamera' というデバイスが見えるようになり、
      それを選択すると、Spresenseのカメラの画像をLinuxの通常のアプリで利用することが出来ます。
