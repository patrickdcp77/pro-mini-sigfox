rappel pour github

git status    pour voir ce qui a changé par rapport à la version précédente
résultat en rouge pour les changements

git add       les fichiers en rouge
git status    les fichiers sont alors verts
git commit -m "le message"  pour créer un paquet qui va être poussé vers github
git push -u origin main   pour envoyer vers la banche main via le mot clef origin qui désigne l'adresse SSH de github

git remote -v
    origin  git@github.com:patrickdcp77/pro-mini-sigfox.git (fetch)
    origin  git@github.com:patrickdcp77/pro-mini-sigfox.git (push)

git remote   pour savoir quel mot clef représente l'adresse (ici en SSH)
    origin

git clone  git@github.com:patrickdcp77/pro-mini-sigfox.git    pour rapatrier à nous tout le dossier de github
