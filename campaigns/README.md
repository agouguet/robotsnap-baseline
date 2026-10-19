# campaigns

Les campagnes ne vivent pas ici. Une campagne est une suite de scénarios à
rejouer, et c'est le **package du cœur** qui la définit et la mesure :

* ses suites sont des fichiers sous `robotsnap/configs/benchmarks/` (`basic`,
  `basic_short`, `smoke`) ;
* `python -m robotsnap benchmark` la rejoue, écrit le relevé et compare deux
  campagnes avec `--compare`.

Ce dossier-ci reste vide : une méthode n'ajoute que son intégration, et la
mesure reste à un seul endroit, pour qu'une campagne rejouée avec l'outil du
cœur ne puisse pas diverger d'une campagne décrite ici.

Voir `robotsnap/docs/manual.md`, section « Benchmark a suite », pour mesurer une
méthode qui pilote le robot elle-même : `--transport ros2 --policy external`.
