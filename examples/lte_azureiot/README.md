# examples/lte_azureiot

This application uses an LTE network connection to perform the following Azure Iot functions:

- Send and receive messages
- Upload and download files

##### prepare:

###### AZURE_IOT_RESOURCE_FILE

File defined by 'AZURE_IOT_RESOURCE_FILE' in "lte_azureiot_main.c".

("/mnt/sd0/azureiot/resources.txt" by default)

Please refer to the URL below and describe "IoT Hub Name", "Device ID", and "Primary key" in order.

https://docs.microsoft.com/en-US/azure/iot-hub/iot-hub-create-through-portal

```
<IoT Hub Name>
<Device ID>
<Primary Key>
```

####### Description example:

This is an example, the contents of resource.txt when "IoT Hub Name" is "iot-hub-contoso-one", "Device ID" is "myDeviceId", and "Primary Key" is "HZAww1PN3suNBkailQU1UeEllNB3j0=" are described.

```
iot-hub-contoso-one
myDeviceId
HZAww1PN3suNBkailQU1UeEllNB3j0=
```

###### AZURE_IOT_CERT_FILE_NAME

File defined by 'AZURE_IOT_CERT_FILE_NAME' in "lte_azureiot_main.c".

("/mnt/sd0/CERTS/portal-azure-com.pem" by default)

Please download certification file as "portal-azure-com.pem" from Microsoft Azure Portal.
(This step is just for firefox web browser.)

1. Open the "https://portal.azure.com" in the Firefox

2. Open the certification details (Baltimore CyberTrust Root)

3. Download the certification file (portal-azure-com.pem)

##### Build kernel and SDK:

This application can be used by lte_azureiot default config.

```
$ ./tools/config.py examples/lte_azureiot
$ make
```

##### Execute under nsh:

  Type 'lte_azureiot \<command\>' on nsh like this.

```
nsh> lte_azureiot recv
```

### Examples:

##### Send 'Hello.' To Azure Iot Hub

```
nsh> lte_azureiot send "Hello."
```

##### Receive "My name is Azure." Message from Azure Iot Hub

```
nsh> lte_azureiot recv
My name is Azure.
```

##### Upload the "HelloStrage.txt" file of the SD card to AzureStrage as "HelloStrage.txt"

```
nsh> lte_azureiot upload /mnt/sd0/Hello.txt HelloStrage.txt
```

##### Download AzureStrage "HelloStrage.txt" file to SD card as /mnt/sd0/Hello.txt

```
nsh> lte_azureiot download HelloStrage.txt /mnt/sd0/Hello.txt
```
