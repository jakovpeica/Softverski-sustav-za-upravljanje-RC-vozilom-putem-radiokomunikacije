# Softverski-sustav-za-upravljanje-RC-vozilom-putem-radiokomunikacije
Opis projekta
Ovaj projekt predstavlja softversko-hardverski sustav za upravljanje RC vozilom pomoću računala i radiokomunikacije. Upravljački ulazi s kontrolera obrađuju se u Python aplikaciji te se bežično prenose na vozilo korištenjem CRSF protokola. Projekt je razvijen u edukativne i natjecateljske svrhe.



Preduvjeti
Hardver
-Računalo (Windows)

-Kontroler (testirano s PS5 kontrolerom putem Bluetootha)

-RadioMaster Ranger Micro (USB veza s računalom)

-RadioMaster RP1 prijamnik

-Arduino mikrokontroler s odgovarajućim programom

-RC vozilo s ispravnim napajanjem


⚠️ Sustav se ne smije koristiti s praznom ili nestabilnom baterijom vozila.

Softver i biblioteke
Za rad aplikacije potreban je Python (preporučeno 3.10+) i sljedeće biblioteke:


pygame

pyserial

PyQt5

pyinstaller 6.18.0

pyinstaller-hooks-contrib 2026.0

altgraph 0.17.5

pefile 2024.8.26

pywin32-ctypes 0.2.3


Instalacija:


pip install pygame pyserial PyQt5

pip install pyinstaller pyinstaller-hooks-contrib altgraph pefile pywin32-ctypes

Pokretanje

-Spojiti kontroler na računalo putem Bluetootha


-Spojiti RadioMaster Ranger Micro na računalo putem USB-a

-Pokrenuti aplikaciju (python rc_app2.py ili .exe verziju)

⚠️ Ako se aplikacija ne pokreće, provjeriti je li kontroler povezan s računalom.

Arduino dio
Arduino mora imati učitan program koji:

-prima CRSF podatke

-dekodira RC kanale

-upravlja servomotorima i ESC-om

-postavlja sigurno stanje u slučaju gubitka signala


Bez odgovarajućeg Arduino koda sustav neće raditi ispravno.

Napomene


Projekt je u razvojnoj fazi. Zbog ograničene dostupnosti hardvera nisu sve funkcionalnosti testirane na konačnom vozilu, no osnovne funkcije upravljanja, komunikacije i obrade ulaza rade ispravno.
