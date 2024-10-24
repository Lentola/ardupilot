import MAVLink
from MissionPlanner.Utilities import Locationwp
import math
import clr
import time

clr.AddReference("MissionPlanner.Utilities")  # includes the Utilities class
clr.AddReference("MAVLink")  # includes the Utilities class

idmavcmd = MAVLink.MAV_CMD.WAYPOINT
id = int(idmavcmd)

# Säädä näitä jos siirtymä heittää
# Esim jos tyynellä siirtymän lakipiste menee pitkäksi, vähennä koneen nopeuden vaikutusta
# Toinen esim jos tuulisella menee lyhyeksi, lisää tuulen voiman vaikutusta
koneen_nopeuden_vaikutus_vtol_siirtymaan = 1
tuulen_voiman_vaikutus_vtol_siirtymaan = 2


############### GLOBAALEJA VAKIOITA ###################
wp_lista_kopio = []

# wp_total = kaikki pisteet + koti, eli yksi piste enemmän kuin mitä kartalla on
# vähennetään jotta ei tule overflow-virhettä
wp_total = int(MAV.getWPCount()) - 1

DO_SET_SERVO = 183
WAYPOINT = 16
DO_VTOL_TRANSITION = 3000
VTOL_LAND = 85

# Muunnoskerroin asteista metreiksi
leveysaste_per_metri = 1 / 53019
pituusaste_per_metri = 1 / 111111

############### GLOBAALEJA MUUTTUJIA ###################
# Tätä listaa kone käyttää. Siihen päivitetään uudet koordinaatit
wp_lista = []

############### LUODAAN PISTEISTÄ TAULUKKO ########################


def luo_taulukko():

    # Otetaan tiedot talteen
    # +1 koska otetaan kotipiste mukaan
    for i in range(wp_total +1):

        wp_get = MAV.getWP(i)

        wp_temp = {
            "lat": wp_get.lat,
            "lng": wp_get.lng,
            "alt": wp_get.alt,
            "id": wp_get.id,
            "p1": wp_get.p1,
            "p2": wp_get.p2,
            "p3": wp_get.p3,
            "p4": wp_get.p4,
        }
        # Ladataan tiedot listaan
        wp_lista.append(wp_temp)
        wp_lista_kopio.append(wp_temp)

    # Tehdään taulukoista wp-muotoinen
    for i, wp_data in enumerate(wp_lista):
        wp_lista[i] = Locationwp().Set(
            wp_data["lat"], wp_data["lng"], wp_data["alt"], wp_data["id"]
        )
        Locationwp.p1.SetValue(wp_lista[i], wp_data["p1"])
        Locationwp.p2.SetValue(wp_lista[i], wp_data["p2"])
        Locationwp.p3.SetValue(wp_lista[i], wp_data["p3"])
        Locationwp.p4.SetValue(wp_lista[i], wp_data["p4"])
    print("Taulukot luotu!")


############### LADATAAN PISTEET KARTALLE #################


def lataa_kartta():
    #+1 koska nyt halutaan huomioida kotipiste
    MAV.setWPTotal(wp_total +1)
    for i in range(wp_total +1):
        MAV.setWP(wp_lista[i], i, MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT)
    MAV.setWPACK()
    print("Kartta ladattu!")



######## LASKETAAN PAKETIN NOPEUS X SUUNNASSA ##################


def laske_x_nopeus_asteet(lat, lng, lat_vanha, lng_vanha):
    delta_y = lat - lat_vanha
    delta_x = lng - lng_vanha

    #Muutos metreiksi
    delta_x = delta_x / leveysaste_per_metri
    delta_y = delta_y / pituusaste_per_metri

    hypotenuusa = math.sqrt(delta_x**2 + delta_y**2)
    kulma = delta_x / hypotenuusa
    x_nopeus = kulma * cs.groundspeed

    # Palautetaan nopeuden pituussuunnan itseisarvo
    return x_nopeus


######## LASKETAAN PAKETIN NOPEUS Y SUUNNASSA #################


def laske_y_nopeus_asteet(lat, lng, lat_vanha, lng_vanha):
    delta_y = lat - lat_vanha
    delta_x = lng - lng_vanha

    #Muutos metreiksi
    delta_x = delta_x / leveysaste_per_metri
    delta_y = delta_y / pituusaste_per_metri

    hypotenuusa = math.sqrt(delta_x**2 + delta_y**2)
    kulma = delta_y / hypotenuusa
    y_nopeus = kulma * cs.groundspeed

    # Palautetaan nopeuden pituussuunnan itseisarvo
    return y_nopeus

######### SELVITETÄÄN TUULEN Y KATEETTI #########################

def selvita_tuulen_y_kateetti(tuulen_nopeus, tuulen_suunta):
    #tuulen nopeus = hypotenuusa
    #Etelään = negatiivinen, Pohjoiseen = positiivinen
    if tuulen_suunta <= 90:
      y_kateetti = math.cos(math.radians(tuulen_suunta)) * tuulen_nopeus
      y_kateetti = y_kateetti *-1
    elif 180 >= tuulen_suunta > 90:
     tuulen_suunta -= 90
     y_kateetti = math.sin(math.radians(tuulen_suunta)) * tuulen_nopeus
    elif 270 >= tuulen_suunta > 180:
     tuulen_suunta -= 180
     y_kateetti = math.cos(math.radians(tuulen_suunta)) * tuulen_nopeus
    elif tuulen_suunta >= 270:
     tuulen_suunta -= 270
     y_kateetti = math.sin(math.radians(tuulen_suunta)) * tuulen_nopeus
     y_kateetti = y_kateetti * -1
    return y_kateetti

######### SELVITETÄÄN TUULEN X KATEETTI #########################

def selvita_tuulen_x_kateetti(tuulen_nopeus, tuulen_suunta):
 #tuulen nopeus = hypotenuusa
 #Länteen = negatiivinen, Itään = positiivinen
 if tuulen_suunta <= 90:
     x_kateetti = math.sin(math.radians(tuulen_suunta)) * tuulen_nopeus
     x_kateetti = x_kateetti *-1
 elif 180 >= tuulen_suunta > 90:
     tuulen_suunta -= 90
     x_kateetti = math.cos(math.radians(tuulen_suunta)) * tuulen_nopeus
     x_kateetti = x_kateetti *-1
 elif 270 >= tuulen_suunta > 180:
     tuulen_suunta -= 180
     x_kateetti = math.sin(math.radians(tuulen_suunta)) * tuulen_nopeus
 elif tuulen_suunta > 270:
     tuulen_suunta -= 270
     x_kateetti = math.cos(math.radians(tuulen_suunta)) * tuulen_nopeus
 return x_kateetti


############### ASETETAAN VTOL SIIRTYMÄN PISTE ############

def aseta_vtol_siirtyma_piste(wp_nro):

    #Haetaan edeltävän pisteen koordinaatit
    x = 1
    while wp_lista[wp_nro -x].id != WAYPOINT:
       x += 1
    lat_alku = wp_lista_kopio[wp_nro-x]["lat"]
    lng_alku = wp_lista_kopio[wp_nro-x]["lng"]

    lat_loppu = wp_lista_kopio[wp_nro]["lat"]
    lng_loppu = wp_lista_kopio[wp_nro]["lng"]

    # Lasketaan koneen nopeus suoran alussa, on positiivinen tai negatiivinen suunnan mukaan
    # Nopeus on suoran suuntainen, tällä siis saadaan pisteet suoralle
    alkunopeus_y = laske_y_nopeus_asteet(lat_loppu, lng_loppu, lat_alku, lng_alku)
    alkunopeus_x = laske_x_nopeus_asteet(lat_loppu, lng_loppu, lat_alku, lng_alku)

    tuulen_nopeus = cs.wind_vel
    tuulen_suunta = cs.wind_dir

    #Tuulen x-ja y-kateetit ovat positiivisia tai negatiivisia tuulen suunnan mukaan
    tuulen_y_kateetti = selvita_tuulen_y_kateetti(tuulen_nopeus, tuulen_suunta)
    tuulen_x_kateetti = selvita_tuulen_x_kateetti(tuulen_nopeus, tuulen_suunta)


    #Tässä päätetään koneen nopeuden ja tuulen voiman vaikutus
    pitkittainen_muutos = (
        alkunopeus_y
        * koneen_nopeuden_vaikutus_vtol_siirtymaan
        + tuulen_y_kateetti
        * tuulen_voiman_vaikutus_vtol_siirtymaan)

    poikittainen_muutos = (
        alkunopeus_x
        * koneen_nopeuden_vaikutus_vtol_siirtymaan
        + tuulen_x_kateetti
        * tuulen_voiman_vaikutus_vtol_siirtymaan)

    lat_loppu = wp_lista_kopio[wp_nro+1]["lat"]
    lng_loppu = wp_lista_kopio[wp_nro+1]["lng"]

    lat = lat_loppu - pitkittainen_muutos * pituusaste_per_metri
    lng = lng_loppu - poikittainen_muutos * leveysaste_per_metri

    Locationwp.lat.SetValue(wp_lista[wp_nro], lat)
    Locationwp.lng.SetValue(wp_lista[wp_nro], lng)



############### SUORITA OHJELMA ################

luo_taulukko()
lataa_kartta()


#Varmistetaan että VTOL_LAND on olemassa
vtol_nro = 0
for i in range(wp_total +1):
    if wp_lista[i].id == VTOL_LAND:
        vtol_nro = i
        print("Loydettiin VTOL_LAND")

if vtol_nro == 0:
   print("Virhe, ei loydetty VTOL_LAND!")

x = 1
while wp_lista[vtol_nro -x].id != WAYPOINT:
    x += 1

#Tämä on säädettävä piste
vtol_nro = vtol_nro - x
print("Valittu piste: ", vtol_nro)

wp_nro = int(cs.wpno)

# Odotetaan kunnes ollaan lähempänä VTOL_LAND-kohtaa
while vtol_nro > wp_nro + 5:
    print("Asetetaan VTOL-kiitorata vasta lahempana...")
    time.sleep(2)
    wp_nro = int(cs.wpno)

# Tässä tehdään laskut ja säädöt vtol-siirtyymää varten
if vtol_nro > wp_nro:
    #Säädetään pistettä ennen VTOL_LAND-komentoa
    aseta_vtol_siirtyma_piste(vtol_nro)
else:
    print("VTOL LAND-komentoa edeltavan pisteen valmistelu alkoi liian myohaan!")
    print("Wp_nro: ", wp_nro)
    print("Saadettava piste: ", vtol_nro)

lataa_kartta()

print("Valmis!")