
About postfilter worker
===========================

This directory is a build environment for postfilter worker.
You are able to build postfilter worker and implement as you like.
For example, low pass filter, band pass filter, and any more.

---------------------------
Directorys and files


-main.cpp

  This file include main() funtion.
  Basically, you should not edit this file, but you can edit, too.
  If you edit these files, the operation cannot be guaranteed.

-[lib]

  Including Makefile to build ASMP library which is used for inter CPU communication.
  Source codes are place at SDK directory, and you don't need to change them.
 
-[userproc]
 
  There are source codes for your Postfilter worker.
  Basically, under this directory, all files are able edit by user freely.

  -[include]

    -rcfilter.h
    -userproc.h

      Header files of this sample RC fileter.

    -userproc_command.h

      Command format defition.
      You can change this file to fit your Postfilter worker command.
      But all command must inherit base command definition of framework. 

  -[src]

    Postfilter source codes are there. Only templete is written. 
    You are able to implement as you like.

    -rcfilter.cpp
    -userproc.cxx

      Source code files of this sample RC fileter.

---------------------------
Build

Postfilter worker build will be automatically run when build this example.
When build complete "POSTPROC" elf binary file will be created "worker" directory.

