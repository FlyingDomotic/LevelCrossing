#Level crossing for model railroad/Passage à niveau pour train miniature

[Cliquez ici pour la version française plus bas dans ce document](#france)

# <a id="english">English version</a>
This code manages a level crossing for model railroad.

Level crossing has one barrier on each side, in front of car road side. It may also have  opposite side, which will close after car side, giving a delay.

Settings are send to Arduino through its serial (USB) link. This link is also used to send feedback and messages to user.

Settings are saved in Arduino's EEPROM to be available after (re)start.

# Principle:
    A pair of detectors (ILS, infrared, contact, hall effect detector...) is put each side of level crossing to manage. When a train is detected, we start blinking LEDs, sound ring if MP3 player installed, wait a bit before closing first set of barriers (those not on car road side), wait a bit before closing second set of barriers (in case of a 4 barriers level crossing), and wait for train leaving zone to reopen at the same time all barriers, stop blinking, stop sound if used.

    Train entering is done when entry detectors are activated for a given time (to avoid false detection).

    Train leaving is done when leaving detectors are not activated anymore for a given time.

    To be realistic, entering detector should be put something far of level crossing, to let time for level crossing to blink/sound/close barriers, while leaving detector should be close to level crossing, to quickly reopen road when train is gone.

    Usually, a track is one way only, but you may want to protect a level crossing in both directions. In this case, use a second pair of detectors in the opposite directions.

    Please note that track is considered to be free if a train is seen in reverse direction as reverse direction is only here to avoid closing barriers when loco hits enter ILS after leave.

    If you wish to protect reverse direction on a track, just put a couple of ILS in reverse direction

    You may protect up to 5 directions:
        - 5 one way tracks,
        - 1 two ways tracks and 3 one way track,
        - 2 two ways track, and one one way track

# Hardware Arduino Nano:
    - 2 servomotors:
        - to manage barriers on car side of the road (will close first)
    - 2 optional servo motors:
        - to manage barriers on opposite side of the road (will close after the car side if present)
    - 4 LEDS:
        - 2 on each side of level crossing, will blink alternatively
    - 1 IR detector/ILS/switch/hall effect sensor per track and traffic direction:
        - to detect train entering into protected zone
    - 1 IR detector/ILS/switch/hall effect sensor per track and traffic direction:
        - to detect train leaving into protected zone
    - 1 optional DY-SV17F sound module:
        - to ring when train is detected
    - 1 optional DCC decoder:
        - to remotely control level crossing and manual/automatic mode. Note that DCC only command is possible. In this case, you may supress detectors if train detection is done outside.

# Setting parameters:
    - define open and close angles for the 2 barriers on car side/full side
    - define delay to add after a degree rotation (set servo speed)
    - define delay to wait after detecting train before closing barriers
    - if 2 barriers on each road side (else let the 4 angles to zero):
        - define open and close angles for the 2 barriers on opposite side
        - define delay to wait after closing those on car side (ms)
    - define LEDs blink time (ms)
    - define time to wait with IR detection before considering it as used (ms)
    - define time to wait without IR detection before considering it as unused (ms)
    - if sound is used:
        - set volume
        - set sound index
    - if DCC decoder is used:
        - set manual mode address
        - set open/close address

# Direction state management:
    In normal direction: when detect train entering (on enter ILS), wait for first exit, then wait leaveTrainDelay after last exit (on leave ILS)
        - if enter ILS just closed and state is waitingForTrain -> set state to waitingForFirstExit and close barriers
        - if leave ILS just closed ans state is waitingForFirstExit -> set state to waitingForLastExit
        - if leave ILS is opened and state is waitingForLastExit -> arm exit timer
        - when exit timer expires -> set state to waitingForTrain, if all directions are in waitingForTrain or waitForLastReverseExit state then open barriers
    In  reverse direction: when detect train entering in reverse way (on leave ILS), wait leaveTrainDelay after last exit (on enter ILS)
        - if leave ILS just closed and state is waitingForTrain -> set state to waitForLastReverseExit
        - if enter ILS just opened and state is waitForLastReverseExit -> arm exit timer
        - same exit timer expiration as normal direction

# Commands:
    - B1CA0-180 : Barrier 1 close angle
    - B1OA0-180 : Barrier 1 open angle
    - B2CA0-180 : Barrier 2 close angle
    - B2OA0-180 : Barrier 2 open angle
    - B3CA0-180 : Barrier 3 close angle
    - B3OA0-180 : Barrier 3 open angle
    - B4CA0-180 : Barrier 4 close angle
    - B4OA0-180 : Barrier 4 open angle
    - WBC0-9999 : Wait before close (ms)
    - DFED0-99 : Delay for each degree (ms)
    - DBSC0-9999 : Delay before second close (ms)
    - LBT0-9999 : LED blink time (ms)
    - DT0-9999 : De-bounce time (ms)
    - LTD0-9999 : Leave train delay (ms)
    - DCC0-1 : Use DCC
    - MADA0-2044 : Manual mode DCC address
    - OBDA0-2044 : Open barriers DCC address
    - CBDA0-2044 : Close barrier DCC address
    - CS0-99 : Closing sound
    - TS1-99 : Test sound
    - SV0-30 : Sound Volume
    - SI0-999 : Sound Increment (ms)
    - R : Run
    - S : Stop
    - IS : ILS State
    - O : Open barriers
    - C : Close barriers
    - D : Toggle Debug
    - INIT : Global Initialization
    - DV : Display Variables

Author : Flying Domotic, April 2025, for FabLab
License: GNU GENERAL PUBLIC LICENSE - Version 3, 29 June 2007

# <a id="france">Version française</a>	
Ce code gère un passage à niveau pour train électrique miniature.

Le passage à niveau a une barrière de chaque côté de la voie, du côté où roulent les voitures. Il est également possible de mettre une barrière de l'autre côté de la route, qui sera fermée après celle côté voitures après un délai.

Les paramètres sont définis au travers du lien série (USB). Ce lien est aussi utilisé pour envoyer des messages à l'utilisateur.

Les paramètres sont enregistrés dans l"EEPROM de l'Arduino pour être disponibles après un (re)démarrage.

# Principe :
    Une paire de détecteurs (ILS, infrarouge, contact, effet Hall) est installé de chaque côté du passage à niveau à contrôler. Lorsqu'un train des détecté, on allume alternativement des LEDs, fait entendre une sonnerie si un module MP3 est installé, on attend  un peu avant de fermer les barrières côté voitures, attend encore un peu avant de fermer les barrières du côté opposé de la route (dans le cas d'un passage à 4 demi barrières), et attend enfin que le train quitte la zone avant d'ouvrir en même temps les barrières, arrête le clignotement, et le son si utilisé.

    L'entrée du train est validé lorsque les détecteurs sont activés pendant une durée donnée (pour éviter de fausses détections).

    La sortie du train est effective lorsque les détecteurs de sortie ne sont plus activés pendant un temps donné.

    Pour être réaliste, les détecteurs d'entrée ne doivent pas être pas placés trop près du passage à niveau pour laisser le temps au système de faire clignoter les LEDs, activer le son et fermer les barrières, alors que les détecteurs de sortie doivent être placés près du passage à niveau, pour ré-ouvrir les barrières juste derrière le train.
    
    Les voies sont habituellement à sens unique, mais vous pouvez vouloir protéger une voie dans les deux directions. Dans ce cas, utilisez une seconde paire de détecteurs dans la direction opposée.

    Notez qu'une voie est considérée comme libre si un train est vu dans la direction opposée, parce que la détection de sens opposé est là juste pour éviter de refermer le passage lorsqu'une loco en sens inverse passe sur le détecteur de d'entrée après le détecteur de sortie.

    Si vous voulez protéger la direction inverse, placez simplement un second couple de détecteurs.
    
    Vous pouvez protéger jusqu'à 5 directions :
        - 5 voies à sens unique,
        - 1 voie à double sens et 3 voies à sens unique,
        - 2 voies à double sens et une voie à sens unique.

# Matériel pour un Arduino Nano :
    - 2 servomoteurs:
        - pour gérer les barrières en face la route côté voitures (vont être fermés en premier)
    - 2 servomoteurs optionnels :
        - pour gérer les barrières du côté opposé des voitures (vont être fermés après le côté voitures si présents)
    - 4 LEDs:
        - 2 de chaque côté du passage à niveau, qui clignoteront alternativement
    - 1 détecteur infrarouge/ILS/contact/détecteur à effet Hall par voie et direction à protéger :
        - pour détecter un train entrant dans la zone à protéger
    - 1 détecteur infrarouge/ILS/contact/détecteur à effet Hall par voie et direction à protéger :
        - pour détecter un train quittant dans la zone à protéger
    - 1 module son DY-SV17F optionnel :
        - pour faire entendre une sonnerie
    - 1 décodeur DCC optionnel :
        - pour contrôler à distance le passage à niveau (mode manuel/automatique, ouverture et fermeture). Noter qu'il est possible de ne commander le passage qu'en DCC, et de supprimer les détecteurs si la gestion des cantons est réalisée à l'extérieur.

# Définition des paramètres :
    - définir les angles d'ouverture et de fermeture des 2 barrières côté voiture de la route
    - définir le délai à ajouter après chaque degré de rotation (pour ralentir la barrière)
    - définir de délai à attendre après la détection du train avant de fermer les barrières
    - si on utilise 2 barrières de chaque côté de la route :
        - définir les angles d'ouverture et de fermeture des 2 barrières côté opposé de la route
        - définir le délai d'attente entre la fermeture des barrières côté voitures et côté opposé (ms)
    - définir le temps d'allumage des LEDs (ms)
    - définir le temps d'attente avant de considérer un changement d'état d'un détecteur (anti-rebond, ms)
    - définir le temps d'attente avant de considérer le train comme sorti après l'ouverture du détecteur de sortie (ms)
    - si on a installé un module son :
        - définir le volume sonore
        - définir l'index du son
    - si on a installé un décodeur DCC :
        - définir l'adresse DCC du mode manuel
        - définir l'adresse DCC de la commande d'ouverture
        - définir l'adresse DCC de la commande de fermeture

# Gestion de la direction de la voie :
    Dans le sens normal : lorsqu'on détecte une entrée de train (sur le détecteur d'entrée), on attend le premier passage sur le détecteur de sortie, puis on attend la sortie du dernier wagon (sur le détecteur de sortie) :
        - si le détecteur d'entrée est juste fermé et l'état est waitingForTrain, alors mettre l'état à waitingForFirstExit et fermer les barrières
        - si le détecteur de sortie est juste fermé et que l'état est waitingForFirstExit, alors mettre l'état à waitingForLastExit
        - si le détecteur de sortie est juste ouvert et que l'état est waitingForLastExit alors armer le délai de sortie
        - lorsque le délai expire alors mettre l'état à waitingForTrain et si toutes les directions sont dans l'état waitingForTrain ou waitForLastReverseExit alors ouvrir les barrières
    Dans le sens opposé : lorsqu'on détecte une entrée de train dans le sens opposé (sur le détecteur de sortie), aon attend la sortie du dernier wagon (sur le détecteur d'entrée)
        - si le détecteur de sortie est juste fermé et que l'état est waitingForTrain alors mettre l'état à waitForLastReverseExit
        - si le détecteur d'entrée est juste ouvert et que l'état est waitForLastReverseExit alors armer le détecteur de sortie
        - même traitement de l'expiration du délai de sortie que pour le sens normal

# Commandes :
    - AFB10-180 : Angle fermeture barrière 1 (°)
    - AOB10-180 : Angle ouverture barrière 1 (°)
    - AFB20-180 : Angle fermeture barrière 2 (°)
    - AOB20-180 : Angle ouverture barrière 2 (°)
    - AFB30-180 : Angle fermeture barrière 3 (°)
    - AOB30-180 : Angle ouverture barrière 3 (°)
    - AFB40-180 : Angle fermeture barrière 4 (°)
    - AOB40-180 : Angle ouverture barrière 4 (°)
    - AAF0-9999 : Attente avant fermeture (ms)
    - AACD0-99 : Attente après chaque degré (ms)
    - AASF0-9999 : Attente avant seconde fermeture (ms)
    - DCL0-9999 : Durée clignotement LED (ms)
    - DAR0-9999 : Délai anti-rebond (ms)
    - TST0-9999 : Temps sortie train (ms)
    - DCC0-1 : Utiliser DCC
    - ADMM0-2044 : Adresse DCC mode manuel
    - ADOB0-2044 : Adresse DCC ouverture barrières
    - ADFB0-2044 : Adresse DCC fermeture barrières
    - SF0-99 : Son fermeture
    - TS1-99 : Test son
    - V0-30 : Volume son
    - IS0-999 : Incrément son (ms)
    - M : Marche
    - A : Arrêt
    - E : Etat ILS
    - O : Ouverture barrières
    - F : Fermeture barrières
    - D : Bascule déverminage
    - INIT : Initialisation globale
    - AV : Afficher variables

Auteur : Flying Domotic, April 2025, pour le FabLab
Licence: GNU GENERAL PUBLIC LICENSE - Version 3, 29 June 2007
