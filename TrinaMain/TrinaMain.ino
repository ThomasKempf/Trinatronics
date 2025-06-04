void setup() 
{
  setupModbus();
  referenceProtocol();

  pinMode(pinOnOff, INPUT_PULLUP);      // SWITCH défini en entrée
  pinMode(pinButtonMode, INPUT_PULLUP); // BP défini en entrée
  pinMode(pinLedButton, OUTPUT);        // BP : LED définie en sortie
  pinMode(pinRelay, OUTPUT);            // LED : Relais défini en sortie

  Serial.begin(115200);                 // Initialisation du port série
  setupModbus();
  referenceProtocol();
  Serial.println("--- Démarrage du système ---");
}

// example ho to use UpdateControlerStatus
void loop() 
{
  Serial.begin(9600);
  // define the parameter
  x_moteur = 100; // mm
  y_moteur = 100;
  rh = 0; // °*10^-2
  rv = 9000;
  AutoMode = true; // Quand t'es en mode auto il se déplace
  ManualMove = false; // Quand il y a un déplacement en cours (c'est quand c'est appuyé sur les boutons)
  UpdateControlerStatus();
  delay(4000);

  x_moteur = 100; // mm
  y_moteur = 100;
  rh = 0; // °*10^-2
  rv = 9000;
  AutoMode = true; // Quand t'es en mode auto il se déplace
  ManualMove = false; // Quand il y a un déplacement en cours (c'est quand c'est appuyé sur les boutons)
  UpdateControlerStatus();
  delay(4000);

  x_moteur = 700; // mm
  y_moteur = 200;
  rh = 9000; // °*10^-2
  rv = 27000;
  AutoMode = true; // Quand t'es en mode auto il se déplace
  ManualMove = false; // Quand il y a un déplacement en cours (c'est quand c'est appuyé sur les boutons)
  UpdateControlerStatus();
  delay(4000);
}