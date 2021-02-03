# GPS_Tracker_Gopro_ESP8266
tracker gps + commande wifi gopro - esp8266

* balise GPS de signalisation basée sur https://github.com/dev-fred/GPS_Tracker_ESP8266
* Commande gopro basée sur https://github.com/aster94/GoProControl

Penser à intégrer la bibliothèque de aster94 pour que cela puisse fonctionner.
Si des problèmes surviennent pour la connection de la gopro, augmenter le MAX_WAIT_TIME dans le settings.h d'aster94 à 5000, ou plus.

La balise peut fonctionner avec ou sans gopro, le bouton on/off de la gopro est la pour ça. Attention, si le bouton gopro de l'esp est sur "on", mais que la gopro n'a pas le wifi activé, la balise tournera en boucle en cherchant uniquement à se connecter à la gopro. (si c'est le cas, la led rouge va clignoter 3 fois rapidement, s'arrêter, puis recommencer)
Elle fonctionne avec un GPS BN-220.

Les relais sont commandés par commande PWM en sortie de récepteur radio. A configurer sur votre radio.
Je mettrai en ligne ultérieurement les réglages openTX de la radio.

Un délai de 2 secondes (non bloquant, sur timer) est prévu entre deux requêtes faites à la gopro, afin d'être sûr que celle ci recoit bien les ordres et les exécutes.

La led verte (GPS), clignote 1 fois quand la balise trouve un nouveau satellite, ou est en attente de satellites.

# Fonctionnement des LEDs :
En fonctionnement normal :
* Gopro activée + fonctionnement normal de la balise GPS : Leds verte et rouge allumées fixes
* Gopro désactivée + fonctionnement normal de la balise GPS : Led vert allumée, led rouge éteinte
  
Si il y a un problème :
* Led rouge clignote: problème de connection à la gopro
* Led verte clignote: pas assez de satellite, problème sur GPS, ou nouveau satellite trouvé (led verte fixe -> clignote 1 fois -> led verte fixe)

# Schéma de câblage :
![alt text](https://github.com/damiendon/GPS_Tracker_Gopro_ESP8266/blob/main/GPS_Tracker_Gopro_ESP8266-SCHEMA.png "schema de cablage")

# Choix inter radio :
  - L'inter pour le "trigger" (déclenchement photo / début-arrêt video) doit être un interrupteur monostable, inter ecolage par exemple.
  - L'inter pour le choix du mode (photo/video) doit être un inter bistable

# Réglages Radio :

L'inter SG (bistable) sert à controller les modes photo/video
L'inter SH (monostable) sert à controller le déclenchement photo/video

Libre à vous de choisir quels inters vous conviennent le mieux ! Bien garder en mémoire que le "trigger" doit être monostable.

* __Sons à mettre dans le dossier "SOUNDS" de la radio__

https://github.com/damiendon/GPS_Tracker_Gopro_ESP8266/blob/main/GPS_Tracker_Gopro_ESP8266-sons%20opentx.zip

* __Onglet Entrées et mixage__ (exemple)

![alt text](https://github.com/damiendon/GPS_Tracker_Gopro_ESP8266/blob/main/00-onglet%20entr%C3%A9es.png "00-onglet entrée")
![alt text](https://github.com/damiendon/GPS_Tracker_Gopro_ESP8266/blob/main/01-onglet%20mixages.png "01-onglet mixages")

* __Onglet Inter logiques__

![alt text](https://github.com/damiendon/GPS_Tracker_Gopro_ESP8266/blob/main/02-onglet%20inter%20logiques.png "02-onglet inter logiques")

* __Onglet Fonctions spéciales__

![alt text](https://github.com/damiendon/GPS_Tracker_Gopro_ESP8266/blob/main/03-onglet%20fonctions%20sp%C3%A9ciales.png "03-onglet fonctions spéciales")
