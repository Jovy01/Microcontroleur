 l'objectif  de cette manipulation est d'aller dans les bibliothèque du stm32
 et de récuper uniquement les informations dont nous avons besoin pour 
 réaliser une tache spécifique :

 les taches spécifiques ici, mesurer la préssion et les paramètres du gyroscope

  * mesure de la pression

 - dans la bibliothèque main.h on met en commantaire toutes les lignes ne faissant pas 
  reference a la tache que l'on souhaite réaliser

  - dans le main.c oon doit fournir un programme sans bibliothèque (les seules
   bibliothèque que nous avons laisser sont celles de la librairie c "stdio.c" et 
  celle initialisant les périphériques du STM32

  - le principe ici étant de décortiquer les  fonctions les unes après les autres
  à l'aide de la fonction "go to definition" du logiciel STM32 pour pouvoir récuperer 
  et comprendre le role de la fonction
  
  - à chaque fois qu'on recupère une fonction pour l'inserer dans le main, on compile cette 
    dernière pour reperer les erreurs et ensuite chercher dans les bibliothèques les éléments
    d'initialisation ou de reférence de ces derniers.
   
  - chaque fonction fait appel a des éléments se trouvant dans des bibliothèques du STM32, 
    les récupérer et les insérer dans le main permettra de reduire le nombre d'erreur 

   - ainsi, on pourra obtenir dans un seul dossierun programme avec tout les elements nécesaires 
     à son bon fonctionnement.


  - NB: ne pas oublier de bien ranger ces fonction, et la declaration de celles çi 
    
     * mesure des paramètres du gyroscope
  le processus est le meme