Osvetleni v hornim pokoji - kontroler
-----------------------------------------------

Fígl je v tom, že to bude 12V okruh.
Zdroj je 12V/10A a jeho napájení bude řízené přes relátko, aby neběžel naprázdno v mezidobích, kdy nebude svítit žádné světlo (tzn. pokud má svítit aspoň jedno ze tří světel, relátko bude sepnuté, jinak rozepnuté).
Světla budou dohromady 3 - dvě lampičky na šikminách a hlavní světlo bude třímetrovej led pásek cca 36W nahoře na rozpěře střechy a bude svítit nahoru.
Zapínat/vypínat se to bude klikacíma vypínačema - tlačítko (click ON, click OFF) a minimálně to hlavní světlo musí být stmívatelné. Tlačítka použijou vnitřní pullupy z procesoru (tzn. aktivní bude LOW).
Mám nakoupený destičky s MOSFETama, takže asi použiju rovnou tři a do světel to pujde přes ně, jinak bych musel použít stejně tři relátka a k tomu jeden mosfet, tak v sestavě 1relé-+3mosfet to bude asi efektivnější.

Rozsvícení - krátký klik - nájezd na 100%
Pokud je zhasnuto a tlačítko se přidrží (1s), začne se plynule pomalu rozsvěcet od nuly dokud se tlačítko drží, po puštění se nechá aktuální intenzita.
Zhasnutí - krátký klik

Stmívání bude fungovat přidrženim tlačítka (1s) a bude to klasická PWM asi někde okolo 20kHz.
Po zapnutí bude vždy výchozí směr pro stmívání dolu. Potom při dalšim přidržení zase nahoru atd..

Samozřejmě bude napojenej MIRF modul.

Blbý je, že timer0 je použitej na časování všeho včetně MIRFu, ale nejde použít jako zdroj pro PWM, protože tiká po 10ms. A zároveň abysme měli tři separátní PWM, musej se použít další dva čítače, protože každej může obsluhovat jen dva PWM kanály.

Takže buď se udělá PWM softwerově, s tim, že se třeba udělá jen 32 levelů a přerušení od timeru bude každejch 25 taktů (25 * 32 = 800 taktů procíku na celou periodu pwm, což je při 16MHz krystalu přesně 20kHz PWM) a nebo se fakt použijou dva zbylý čítače a nastavěj se na stejnou frekvenci.

