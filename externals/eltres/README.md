# ELTRES SDK

The original ELTRES SDK source codes and documents can be downloaded from the following site.

* https://www.sony-semicon.com/en/eltres/module.html (en)
* https://www.sony-semicon.com/ja/eltres/module.html (ja)

The source codes are ported for Spresense and are provided under the `CXM150x_SDK` directory.

The following boards are supported and can be selected by Kconfig.

* `CONFIG_EXTERNALS_ELTRES_SPEXEL`
  * For [ELTRES SPEXEL board](https://device.risner.jp/products/detail/15)
* `CONFIG_EXTERNALS_ELTRES_ADDON`
  * For [ELTRES Add-on board](https://www.cresco-dt.co.jp/service/iot/iot-poc/eltres/)
* `CONFIG_EXTERNALS_ELTRES_ORIGINAL`
  * For user original board, please change the pin assignment to match the board.

In addition to the ELTRES SDK, the following example applications are also provided.

* examples/eltres_lpwa: based on LPWA Sample Application
* examples/eltres_standalone: based on Standalone Mode Sample Application

For more information of ELTRES SDK APIs, please refer to the documents on the website.

## NOTICE

* The feature of firmware update is not supported.
  * `CONFIG_EXTERNALS_ELTRES_CTRL_FWUPDATE` and `CONFIG_EXTERNALS_ELTRES_GNSS_FWUPDATE` should be disable.
* The feature of transmission test mode is not supported.
  * `CONFIG_EXTERNALS_ELTRES_TEST_MODE` should be disable.

## LICENSE

Copyright 2021, 2022 Sony Semiconductor Solutions Corporation

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of Sony Semiconductor Solutions Corporation nor the names of
its contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
