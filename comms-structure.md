# RF Comms Structure

Comms information used for nRF24L01 devices.

## Addresses

-   Tx (vehicle side): 0x01
-   Rx (ground side): 0x02

## Message Format (Assuming GPS Data):

{Velocity1, Velocity2, Alt1, Alt2, Course1, Course2, Dist1, Dist2, Temp1, Temp2, Humid1, Humid2}

-   Velocity: Vehicle velocity as captured via GPS. Unit: m/s
-   Altitude: Vehicle altitude as captured via GPS: Unit: m
-   Course: Vehicle course over the ground as captured via GPS. Unit: Degrees
-   Distance: Vehicle's distance from "home base" (Comparing first captured coordinates with most recent). Unit: m
-   Temperature: Air temperature at vehicle's location as captured by SHT45. Unit: &deg;C
-   Humidity: Relative humidity at vehicle's location as captured by SHT45. Unit: %

## Encoding

**Everything is 2 bytes encoded as follows:**

-   Take the absolute value of the number (we are only considering magnitudes here)
-   Take float, multiply by 100 and cast to uint16_t
-   Split into two bytes

_Note: this results in a conscious decision that any value greater than ~500 will not be encoded and will instead be sent as 0xffff and should be ignored._

You may be thinking: gosh, there must be a better way than to make up this arbitrary encoding scheme for most of these numbers, losing precision like crazy.
You're probably right, but I would rather consistency over the air than to use the 50 different ways these get encoded by the sensor breakout boards.

It was really tempting to break this standard with the temp & humidity values since I'm already reading two raw bytes... but consistency.
