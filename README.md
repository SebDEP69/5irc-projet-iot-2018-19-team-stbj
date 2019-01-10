# 5irc-projet-iot-2018-19-team-stbj

# Projet IoT

Ce projet a pour but de créer le système d'une montre connectée, qui servira à faire des relevés de taux de CO2 via un système de géolocalisation GPS. 
Ces données seront alors envoyées et mise en forme pour analyse et des actions pourront être réalisées pour faire baisser le taux de pollution. 

## Description du Projet

Voici l’organisation de notre projet cette semaine, pour rappel, les trois principales composantes d'un système IoT sont :
Capteurs --> Passerelles --> Cloud
Dans le rôle de capteurs, nous aurons un capteur de CO2 et une balise GPS connectés sur un Arduino. Ce dernier transmettra ces données via ZigBee sur un autre Arduino qui fera la passerelle vers notre BeagleBone. Le BBB se chargera de faire la passerelle vers Ubidots qui est la plateforme Cloud que nous avons choisit d'utiliser.

### Matériel utilisé 

Le capteur C02 utilisé pour ce projet est le MQ-135 sensor. 
La balise GPS est une GPS3 click. 
Les Arduino utilisés sont des Arduino Mega 2560 click shield. L'avantage de ces derniers est qu'ils possèdent plusieurs sérial qui permettent la connexion de trois capteurs par Arduino. 

## Procédure de mise en place de votre chaîne IoT

-----------------------------------------------Capteur --> Arduino-------------------------------------------------------

---------------------------------------------Passerelle --> BeagleBone--------------------------------------------------- 

1 - Connexion au BeagleBone 

Premièrement, téléchargez la version Debian 8.6 (disponible à l'adresse http://beagleboard.org/latest-images).

Flashez votre carte SD avec l'image téléchargée précédemment. Pour cela, vous pouvez utilisé Etcher qui est disponible sur tous les OS (https://www.balena.io/etcher/).

Ensuite, nous vous conseillons de suivre le tutoriel d'installation sur le site http://beagleboard.org/getting-started. 
Afin que votre ordinateur puisse reconnaitre le BeagleBone, il faut installer les drivers adéquates en fonction de votre OS. 

Vous pouvez maintenant connecté votre Beaglebone en USB sur votre ordinateur. A ce moment la, allez dans vos périphériques réseaux, vous verrez une deuxième carte réseau apparaitre. 

Activez le partage de votre connexion internet sur cette nouvelle carte, et configurez son IP pour qu'elle soit dans le meme réseau que votre BeagleBone. 

Petit rappel du tutoriel : 

Sur MACOS X, Linux, le BeagleBone prend l'adresse IP : : 192.168.6.2 et IP de votre carte réseau : 192.168.6.1
Sur Windows, le BBB prend l'adresse IP : 192.168.7.2 et IP de votre carte réseau : 192.168.7.1

En SSH, connectez vous sur le BBB. 
Login : Debian 
Password par défaut : temppwd 

Vous voilà connecté au BBB. 

2 - Configuration du BBB 

A) Vérification de la connexion Internet 
Pour l'instant, votre BeagleBone n'est pas connecté à Internet (test ping 8.8.8.8 failed). Pour configurer la connexion Internet, je vous conseil de suivre ce tutoriel : https://www.digikey.com/en/maker/blogs/how-to-connect-a-beaglebone-black-to-the-internet-using-usb 

Deux choses sont importantes : 
- Il faut ajouter une route par défaut pour dire au BBB par où sortir vers Internet : 

"sudo /sbin/route add default gw 192.168.7.1" (ou "sudo /sbin/route add default gw 192.168.6.1" pour MAC OS/Linux) 

- Dans ce cas là, la connexion est établie mais le DNS ne marche pas. Avec cette commande, vous mettez à jour le DNS dans resolv.conf. 

"echo "nameserver 8.8.8.8" >> /etc/resolv.conf" 
et sur l'interface Ethernet :  
"sudo echo "dns-nameservers 8.8.8.8" >> /etc/network/interfaces"

NB : pensez bien à configurer le DNS sur votre carte Ethernet dans votre PC (ajoutez 8.8.8.8). 

Pour tester, "ping google.com".

B) Configuration des pins 

Pour commencer, nous avons besoin  d'activer les ports UART. 
Pour cela, éditer le fichier /boot/uEnv.txt et ajoutez la ligne suivante :

"cape_enable=bone_capemgr.enable_partno=BB-UART1,BB-UART2" 

et supprimez : 
"cape_universal=enable" 

et redémarrez votre BBB : ce fichier est un fichier système, le BBB ne le prendra en compte que lors du rédémarrage. 
NB : Vérifiez que la connexion internet est toujours présente. Dans le cas contraire, répéter les actions du 1-. 

Vérifications :

Vérifier que les slots sont identifiés:
debian@beaglebone:/sys/devices/platform/bone_capemgr# cat slots
 0: PF----  -1 
 1: PF----  -1 
 4: P-O-L-   0 Override Board Name,00A0,Override Manuf,BB-UART1
 5: P-O-L-   1 Override Board Name,00A0,Override Manuf,BB-UART2
 
Vérifiez que les ttyO sont identifiés:

debian@beaglebone:/dev# ls ttyO*
/dev/ttyO0 /dev/ttyO1 /dev/ttyO2 

Les ports sont maintenant prêts à être utilisés. Dans notre cas, nous utiliserons que les UART1 et UART2. 

C) Interconnexion physique 

Nous allons maintenant connecter physiquement les ports UART sur le BeagleBone pour vérifier qu'ils sont bien utilisables. 
Nous allons connecter :  UART1 TXD (P9_24) à UART2 RXD (P9_22) et UART2 TXD (P9-21) à UART1 RXD (P9_26). 

D) Vérification avec Minicom

Ensuite, installez minicom : 

sudo apt-get update 
sudo apt-get install minicom

En root : 
minicom -D /dev/ttyO1 -b 9600 
minicom -D /dev/ttyO2 -b 9600

Deux terminal s'ouvrent alors. Si les ports sont montés, vous pourrez écrire dans le premier et cette écriture se répercutera dans le second. 

E) Connexion avec l'Arduino relais 

Nous allons maintenant connecter le BeagleBone à l'Arduino qui joue le rôle de relais (qui contient le module ZigBee ne réception). 
Pour cela, il faut connecter le pin TX P9.21 sur le sérial 3 port PJ1/TX3 de l'Arduino et le RX P9.22 sur le sérial 3 port PJ0.RX3 de l'Arduino (cf interco.jpg dans le dossier "Images"). 

F) StartGateway 

Pour initier la communication entre le BeagleBone et le cloud, nous avons mis à disposition le fichier start-gateway.py.
Ce fichier se trouve dans le dossier gateway-code. 
Les seules choses à changer sont la variable "TOKEN" (cf partie suivante) correspondant à votre compte Ubidots. 


----------------------------------------------Cloud --> Ubidots---------------------------------------------------------

Nous allons maintenant afficher les données sur le Cloud. 

Pour cela, veuillez vous connecter sur https://industrial.ubidots.com/. 

Il faut créer un compte. Ensuite, pour récupérer le "TOKEN" (qui représente l'identifiant unique de votre compte, il faut cliquer sur votre profil en haut à droite puis, "Api Credentials" et là le TOKEN s'affichera. Vous n'avez plus qu'à le copier-coller dans la variable "TOKEN" du fichier start-gateway.py. 

Ajoutez maintenant les dashboards que vous voulez sur Ubidots. 
Vous verrez dans l'onglet "Devices" les données qui sont envoyées. 






