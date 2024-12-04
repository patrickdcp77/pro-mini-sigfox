# pro-mini-sigfox, extrait du code pour comprendre le fonctionnement du système

fonctionne correctement

+++++++++++++++++++++++++++++++++++++++++++++++

code rédigé pour 
4 balances et une sonde temp hum DHT22

mesure et hibernation tous les 1/4H
envoi sur réseau sigfox

pour les balances, cablage des connecteurs 

utiliser les canaux B des HX711 si l'on veut économiser les convertisseurs


cordon crème et plat

noire   A,B+

rouge   E-

jaune   E+

vert    A,B-


cordon blanc et plat avec 4 fils couleurs pales

rose    E+

bleu    E-

blanc   AB-

jaunes  AB+




correspondance avec les cables noirs étanches

vert    A,B+

noir    E-

rouge   E+

bleu    A,B-


cablage des balances vues en dessous (donc platine porte jauges retournée)

vue eu U inversé

    -------------------------
    
    |  E-              A,B+ |
    
    |                       |  
    
    |                       |
    
    |   A,B-            E+  |
    

fils noirs des jauges relient les jauges E- avec AB-  et AB+ avec E+

fils blancs des jauges telient E- avec AB+    et AB- avec E+


consignes pour programmation du minipro arduino qui n'a pas de schip USB via TX RX pour baisser la consommation

utiliser un FTDI externe qui adaptera le TX RX à l'USB

utiliser la position 3V3

déconnecter le TX RX de sigfox pour ne pas perturber le TX RX du promini

mettre la valeur **** boolean AUTODETECT_HX711_N34 = 0**** ; à UN pour observer sur le terminal les valeurs données par les capteurs

mettre la valeur ****const unsigned int MAX_COUNTER_POWER_DOWN_WAKE_UP = 116 ;**** à 2 ou 3 pour ne pas attendre un cycle de 15mn entre chaque mesure

ne rien mettre sur les balances avant chaque RESET et voir l'évolution des valeurs selon les poids posés

si tout semble bon, remettre les valeurs à ZERO pour ne pas débugger et à 116 pour avoir un cycle de 15mn entre chque envoi de trame sigfox


mise en mémoire de la tare (offset) tant qu'il n'y a pas de reset


pas de DHT22 sur les derniers prototypes

sinon  alime+ de la DHT sur D12  et DATA sur A3


pas de panneau solaire mais utilisation d'une pile plate 4V5 alcaline d'environ 3000 mA

l'alimentation se fait via le connecteur latéral du PRO MINI  VCC et GND

modifications sur PRO MINI:

régulateur enlevé

résistance enlevée de la led de visualisation de l'alimentation

mesure de la tension batterie appliquée sur VCC

on met une résistance CMS de 47K entre A0 et A1 coté supérieur du processeur


on met une résistance CMS de 10K entre A0 et le plan de masse en parallele à un condensateur de 1,1 nano soudés l'un sur l'autre

cablage du module radio sigfox*****************

le reset est à coté du + et connecté sur GPIO 2

le + est sur VCC car on ne peut pas alimenter le module avec un GPIO qui serait trop faible 


cablage du HX711 pour 2 balances sur un seul HX711************

GPIO 3 chaque HX711 a une alimentation commune sur un même GPIO mis à zéro pour couper l'alimentation

GPIO 4 connexion au DATA

GPIO 8 connexion à l'horloge SCK

balance N1_Channel A : connecter sur E+ E- A+ A-

balance N1_Channel B : connecter sur E+ E- B+ B-


Operating voltage:

●2.7V to 5.5V for ATmega328P donc pile plate alcaline de 4V5 possible sans régulateur

Low power consumption

● Active mode: 1.5mA at 3V - 4MHz

● Power-down mode: 1μA at 3V

● Write/erase cycles: 10,000 flash/100,000 EEPROM

