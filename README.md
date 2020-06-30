# Opis projektu

Głównym celem projektu jest stworzenie Joystick'a na bazie mikrokontrolera STM32F103. Założeniem jest aby uniwersalnym urządzeniem można było sterować różnymi robotami przez moduł Bluetooth lub WIFI, oraz tradycyjnie obsługiwać komputer poprzez interfejs USB jako joystick lub jako mysz. Ma on posiadać niżej opisane funkcjonalności:

	* Baterie do pracy bez zasilania przewodowego.
	* Zintegrowaną ładowarkę i kontrolę napięcia baterii.
	* Gałkę precyzyjną i  szereg przycisków do kontroli i sterowania.
	* Akcelerometr do różnego rodzaju kontroli lub jako kolejna “gałka joysticka”.
	* Interfejs USB umożliwiający podłączenie do komputera jako urządzenie HID jak i ładowanie samego urządzenia.
	* Moduł Bluetooth, WIFI i radiowy umożliwiający implementację sterowania różnego rodzaju robotami.
 
![GitHub Logo](/sr_szablon_projekt/figures/calosc1.jpg)

Mikrokontroler STM32F103 przez magistralę UART komunikować się będzie z modułem WIFI i modułem Bluetooth HC-05 aby umożliwić podłączenie i sterowanie innym urządzeniem. Moduł HC-05 jest programowalny (komendami AT można zmienić częstotliwość komunikacji i tryb pracy master/slave). Moduł komunikacji radiowej nRF24L01+ jest zasilany napięciem 1.9 - 3.6 V. Może pracować w trybie nadajnika i odbiornika. Wyświetlacz oled służy do wyświetlania informacji takich jak stan połączenia, informacje zwrotne z urządzenia, stan baterii itp. Przetwornica ma możliwość wyłączenia w przypadku rozładowania baterii, tym zarządza mikrokontroler. Głównym zadaniem będzie działanie jako joystick po podłączeniu do komputera przez USB. Moduł IMU służy jako kontroler tak jak sam joystick. Architektura systemu:

![GitHub Logo](/sr_szablon_projekt/figures/Modules.PNG)

# Opis działania programu

Uniwersalność urządzenia zapewniona jest poprzez możliwość wybierania trybów pracy. Tryb wybierany jest z menu  uruchamianego po włączeniu urządzenia. Do nawigacji po menu służą niebieskie przyciski - skrajne do zmiany opcji, a środkowy do zatwierdzenia. Podstawowym trybem jest praca jako gamepad, dodatkowo jednak na potrzeby projektu robota micromouse dodano tryb pracy jako mostek pomiędzy robotem a komputerem. W miarę potrzeb program można rozszerzyć o kolejne funkcje trybów z odpowiednio skonfigurowanymi peryferiami, w zależności od wymagań projektu. Program urządzenia podzielony jest na funkcje programów i na funkcje główne. Funkcje główne są odpowiedzialne za kontrolę urządzenia i inicjalizację modułów. Funkcje programów dzielą się na dwa typy: inicjalizacji i pętli, inicjalizacja wybranego programu sprowadza się do wywołania funkcji inicjalizujących moduły i~peryferia dostarczone przez funkcje główne, jest to umowny poziom abstrakcji. Funkcja petli programu jest rdzeniem programu, nie powinna ona byc nieskończoną pętlą, gdyz sama będzie wywoływana w takiej pętli, wraz z funkcją Update która odpowiedzialna jest za kontrolę baterii i zasilania. Program został zrealizowany w taki sposób, ponieważ nie można jednocześnie skonfigurować HID i interfejsu szeregowego, dalsze projekty mogły by powodowac inne konflikty. Wybór innego trybu pracy możliwy jest jedynie po zresetowaniu urządzenia.

![GitHub Logo](/sr_szablon_projekt/figures/main.png)
