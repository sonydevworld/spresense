pmic
^^^^

  This command is the utility tool for CXD5247 PMIC

    * CONFIG_SYSTEM_PMIC - Enabled the pmic command

  Example: Show help message

    nsh> pmic -h
    
    Usage: pmic [-h] [-l] [-e <target>] [-d <target>]
                [-r <addr>] [-w <addr> -v <value>]
    
    Description:
     PMIC utility tool
    Options:
     -l: Show power status of the target
     -e <target>: Enable power to the target
     -d <target>: Disable power to the target
     -r <addr>: Single read from <addr>
     -w <addr> -v <value>: Single write <value> to <addr>
     -h: Show this message
    
  Example: Show the power status

    nsh> pmic -l
         Target Name : on/off
         ----------- : ------
              DDC_IO : on
            LDO_EMMC : on
             DDC_ANA : off
             LDO_ANA : on
            DDC_CORE : on
            LDO_PERI : off
                LSW2 : off
                LSW3 : on
                LSW4 : on
                GPO0 : off
                GPO1 : off
                GPO2 : off
                GPO3 : off
                GPO4 : off
                GPO5 : off
                GPO6 : off
                GPO7 : off

  Example: Enable/Disable GPO0

    nsh> pmic -e GPO0
    Enable : GPO0
         Target Name : on/off
         ----------- : ------
              DDC_IO : on
            LDO_EMMC : on
             DDC_ANA : off
             LDO_ANA : on
            DDC_CORE : on
            LDO_PERI : off
                LSW2 : off
                LSW3 : on
                LSW4 : on
                GPO0 : on
                GPO1 : off
                GPO2 : off
                GPO3 : off
                GPO4 : off
                GPO5 : off
                GPO6 : off
                GPO7 : off

    nsh> pmic -d GPO0
    Disable: GPO0
         Target Name : on/off
         ----------- : ------
              DDC_IO : on
            LDO_EMMC : on
             DDC_ANA : off
             LDO_ANA : on
            DDC_CORE : on
            LDO_PERI : off
                LSW2 : off
                LSW3 : on
                LSW4 : on
                GPO0 : off
                GPO1 : off
                GPO2 : off
                GPO3 : off
                GPO4 : off
                GPO5 : off
                GPO6 : off
                GPO7 : off

  Example: Read a value from [61h] register

    nsh> pmic -r 61
    @[61]=>00

  Example: Write a value(55h) to [61h] register

    nsh> pmic -w 61 -v 55
    @[61]<=55

