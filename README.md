# DHT11 driver

Driver for the DHT11 Humidity & Temperature Sensor. Works for Raspberry Pi, only tested on Raspberry Pi 3B.

Thanks to [sysprog21](https://github.com/sysprog21/lkmpg) and 
    [dht11driver](http://www.tortosaforum.com/raspberrypi/dht11driver.htm).

## Installation
```sh
make && make load
```

## Usage
```sh
sudo cat /dev/dht11
```

## Uninstall
```sh
make unload
```
