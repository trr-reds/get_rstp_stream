from PIL import Image

def lire_metadonnees(image_path):
    try:
        img = Image.open(image_path)
        metadata = img.info
        print("Metadata :")
        print(metadata)
        img.close()
    except Exception as e:
        print("Error while reading metadata :", e)

chemin_image = "pgm1/frame000000.jpg"
lire_metadonnees(chemin_image)
