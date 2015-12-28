# README #

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###

This system requires a 4D systems touchscreen, raspberry pi, and https://www.abelectronics.co.uk/p/14/Delta-Sigma-Pi. It can be fitted into a vehicle. It takes in adc values from 8 channels, with 18-bit resolution. Each channel should be voltage divided so as not to damage the system.

* Quick summary
* Version
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###

* Summary of set up
* Configuration
* Dependencies

    geniePi
    https://github.com/4dsystems/ViSi-Genie-RaspPi-Library
    https://github.com/abelectronicsuk/ABElectronics_Python_Libraries

* Database configuration
* How to run tests
* Deployment instructions

    gcc vehicleMon.c adcpiv3.c -o vehicleMon -lgeniePi && ./vehicleMon

### Contribution guidelines ###

* Writing tests
* Code review
* Other guidelines

### Who do I talk to? ###

* Repo owner or admin

    Daniel Robinson
    
    d7robin@gmail.com

* Other community or team contact
