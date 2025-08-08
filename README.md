# BMW-F20-KOMBI-Cluster-Bench-testing
BMW F20 KOMBI Cluster Bench-testing

I had an instrument cluster lying around (from a BMW F20), and I couldn't stand not being able to control it from my laptop, not being able to get a handle on it. And I was jealous of a project of Angus, where he did manage to work with a BMW Gear Selector Stick (from the same car): https://www.projectgus.com/2022/06/bmw-f-series-gear-selector-part-one-failures/. Ultimately, I managed to control most of the panel's functions. I posted it online because I couldn't find much information about it myself, and I wanted to help others who want to play with it get started more quickly.


I used a LILYGO T-CAN485 board https://lilygo.cc/products/t-can485 to send messages to the cluster. The same board is also suitable to function as a CAN bus interface, and is compatible with SavvyCAN, a very useful (and free) tool to analyze CAN bus data. It can make a dump from a CAN bus, but it also can play a recorded dump back to the CAN bus. 
If you want to use a LILYGO T-CAN485 board together with SavvyCAN, have a look at my post here: https://github.com/collin80/SavvyCAN/discussions/941

How does this project work? Make a Arduino IDE project from the supplied ino-file, and upload it to your Arduino/ESP32-board. Connect the instrument cluster to the CAN bus connection of your board, and it will play the right CAN bus frames to get the cluster to show a demo: an acceleration to 260 km/h and back. If you like to see how thats looks on my cluster, this is the link to the youtube video: https://youtu.be/nQFt8mGyW6k

Of course you don't have to use the same board that I used, or even a board at all. You can just have a look at the ino-file and use this as a source for your own project.

The instrument cluster needs a continuous flow of CAB bus data, otherwise it will turn off quite quickly. Furthermore, it needs some reassuring messages from other modules, otherwise all kinds of warnings will appear. I used dumps from the CAN bus of my own car (BMW F45) to gather most of the needed data to get it going.  

There is a lot to improve and also a lot more to tell, but this should be a nice start. Feel free to comment or ask some questions. I am always interested in similar projects from other people.
