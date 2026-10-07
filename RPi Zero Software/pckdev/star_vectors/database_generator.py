import cv2
import numpy as np
import math

def generate_vectors(central_star, surrounding_stars, number_of_vectors):

  #Compute vectors center star - other stars
  vectors_to_center = central_star - surrounding_stars[:number_of_vectors]

  return vectors_to_center

def imagePreprocessing(image):
  #Get mean pixel value from the image
  mean = np.mean(image)
  std = np.std(image)

  _, thresholded = cv2.threshold(image, 1*mean+6*std, 255, cv2.THRESH_BINARY)

  thresholded = thresholded.astype(np.uint8)

  cv2.imwrite('thresholded.jpg', thresholded)
  return thresholded


def findStars(thresholded_image, original_image, number_of_brightest_stars):
    # Find contours (potential stars) on the binary image
    contours, _ = cv2.findContours(thresholded_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    star_data = []

    for cnt in contours:

        # Find the center of the circle enclosing the star
        (cx, cy), radius = cv2.minEnclosingCircle(cnt)

        # Calculate Flux (Total Brightness)
        # 1. Get the bounding box of the contour to create a tiny Region of Interest (ROI)
        x, y, w, h = cv2.boundingRect(cnt)
        roi_original = original_image[y:y+h, x:x+w]

        # 2. Create a local mask for just this bounding box
        roi_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(roi_mask, [cnt], -1, 255, thickness=-1, offset=(-x, -y))

        # 3. Sum the pixel values from the original image where the mask is applied
        flux = roi_original[roi_mask == 255].sum()

        star_data.append({
            'center': [cx, cy],
            'flux': flux
        })

    print("Number of stars found: ", len(star_data))

    # Sort stars by their total flux in descending order (brightest first)
    star_data = sorted(star_data, key=lambda s: s['flux'], reverse=True)

    # Extract the sorted centers
    contour_centers = [s['center'] for s in star_data]
    star_centers = np.array(contour_centers, dtype=np.float32)

    # Select only the top N brightest stars
    brightest_stars = star_centers[:number_of_brightest_stars]

    # Draw on the thresholded image as before
    markStars(brightest_stars, thresholded_image, 'brightest_stars.jpg')

    return star_centers, brightest_stars


def getStarsInRadius(center_star, stars, radius):
  #Filters stars based on a set radius (distance from center star)

  distances = np.linalg.norm(stars - center_star, axis=1)
  #Creates mask to filter stars outside the radius and the central star
  mask = (distances > 1) & (distances <= radius)
  return stars[mask]


def markStars(stars, image, img_name):
  circle_radius = 15

  # Select only the top X largest contours (stars)
  output = image.copy()
  for index, star in enumerate(stars):

      center_x, center_y = int(star[0]), int(star[1])

      # Draw the circle around the star on the output image
      cv2.circle(output, (center_x, center_y), circle_radius, (0, 255, 0), 2)

      # Add the index number next to the star
      cv2.putText(output, str(index), (center_x + 13, center_y - 13), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)

  cv2.imwrite(img_name, output)
  return output



######################################################


vectors_img = cv2.imread("./database_img.tiff", cv2.IMREAD_GRAYSCALE | cv2.IMREAD_ANYDEPTH)
preprocessed_image = imagePreprocessing(vectors_img)
stars, brightest_stars = findStars(preprocessed_image, vectors_img, 15)

polaris = brightest_stars[0]
radius_stars = getStarsInRadius(polaris, stars, 250)

print("Stars around Polaris: ",len(radius_stars))
markStars(radius_stars[:40], preprocessed_image, "polaris_vector.jpg")
markStars(brightest_stars[:15], preprocessed_image, "allstars_vector.jpg")


#Manualy selected stars from the input image
yildun = brightest_stars[1]
ov_cephei = brightest_stars[8]
ursae_minoris_2 = brightest_stars[3]

#Adds individual stars into a single array
ncp_stars = np.stack([yildun, ov_cephei, ursae_minoris_2])

np.save('polaris_vectors.npy', generate_vectors(polaris, radius_stars, 25))
np.save('ncp_vectors.npy', generate_vectors(polaris, ncp_stars, 4))