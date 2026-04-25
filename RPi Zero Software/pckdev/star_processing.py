import cv2
import numpy as np
import math
#import matplotlib.pyplot as plt

#Load database arrays
polaris_database_vectors = np.load("/home/pck/repo/PCK-ST1/RPi Zero Software/pckdev/database/polaris_vectors.npy")
ncp_database_vectors = np.load("/home/pck/repo/PCK-ST1/RPi Zero Software/pckdev/database/ncp_vectors.npy")
star_distances = np.load("/home/pck/repo/PCK-ST1/RPi Zero Software/pckdev/database/star_distances.npy")
star_Polaris_RA_angle = np.load("/home/pck/repo/PCK-ST1/RPi Zero Software/pckdev/database/star_Polaris_RA_angle.npy")
#star_RA_hours = np.load("database/star_RA_hours.npy")

defaultPixPerArcmin = 4056/(14.334*60) #Default pixels per arminute - Later load from config file

def imagePreprocessing(image):
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

  #Get mean pixel value from the image
  mean = np.mean(gray)
  #print("Mean pixel value:", mean)

  #Increase the image contrast based on mean pixel value
  contrast_img = cv2.convertScaleAbs(gray, alpha=1, beta=-mean)

  # Apply Gaussian blur to reduce noise
  blurred = cv2.GaussianBlur(contrast_img, (5, 5), 0)

  # Get threshold from contrasted and blurred image
  #thresh, _ = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
  thresh = np.mean(blurred)

  #print("Initial threshold: ", thresh)
  # Limit the threshold
  min_thresh = 20
  max_thresh = 100
  threshold = max(min_thresh, min(thresh, max_thresh))
  #print("Limited threshold: ",threshold)

  # Apply adaptive thresholding to detect bright spots
  _, thresholded = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY) #If pix value < threshold  --->  turns pixel black

  cv2.imwrite('thresholded.jpg', thresholded)

  return thresholded

def findStars(image, number_of_brightest_stars):
  # Find contours (potential stars)
  contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  # Filter out small contours based on area (to avoid noise)
  min_area = 1  # Minimum area for contours to be considered as stars
  contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
  # Sort contours by area in descending order (for selecting biggest stars)
  contours = sorted(contours, key=cv2.contourArea, reverse=True)

  print("Number of stars found: ",len(contours))

  #top_contours = contours[:number_of_brightest_stars]
  star_centers = np.array([[int(x), int(y)] for (x, y), _ in (cv2.minEnclosingCircle(cnt) for cnt in contours)], dtype=np.int32)

  # Select only the top n brightest stars
  brightest_stars = star_centers[:number_of_brightest_stars]

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

      # Draw the circle around the star on the output image
      cv2.circle(output, star, circle_radius, (0, 255, 0), 2)

      # Add the index number next to the star
      cv2.putText(output, str(index), (star[0] + 13, star[1] - 13), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)

  cv2.imwrite(img_name, output)
  return output

def starCorrelation(star_vectors, database_vectors, threshold, angle_step, debug = False):
  if(len(star_vectors) <=5):
    print("Not enough stars found!")
    return False, None, None;

  star_vectors_unit = (star_vectors / np.linalg.norm(star_vectors, axis=1)[:, np.newaxis]).astype(np.float32)
  database_vectors_unit = (database_vectors / np.linalg.norm(database_vectors, axis=1)[:, np.newaxis]).astype(np.float32)

  cos_threshold = 0.99999 #cos(0.25°) = 0.99999048
  max_matched_vectors = 0
  max_matched_angle = 0

  angles_rad = np.radians(np.arange(-180, 180, angle_step))

  cos_a = np.cos(angles_rad)
  sin_a =  np.sin(angles_rad)

  #Combines sin a cos values into large rotation matrix for all angles
  rot_matrix = np.stack([np.stack([ cos_a, -sin_a], axis=1), np.stack([ sin_a,  cos_a], axis=1)], axis=1).astype(np.float32)

  rotated_vectors = star_vectors_unit @ rot_matrix.transpose(0, 2, 1)               # (angles, star_vectors, 2)
  dots = rotated_vectors @ database_vectors_unit.T                                  # (angles, star_vectors, databese_vectors)
  match_array = np.count_nonzero(dots.max(axis=2) >= cos_threshold, axis=1)  # (N,)

  max_idx = np.argmax(match_array)
  max_matched_vectors = match_array[max_idx]
  max_matched_angle = np.degrees(angles_rad[max_idx])

  #Statistic values
  mean_val = np.mean(match_array)
  std_val = np.std(match_array)
  max_val = np.max(match_array)

  #Z-score calculation
  if std_val > 0:
    z_score = (max_val - mean_val) / std_val
  else:
    z_score = 0

  if debug:
    #Potting results
    """
    angles = np.degrees(angles_rad)
    plt.figure(figsize=(10, 5))
    plt.plot(angles, match_array, label='Match Count')
    # Plot a "Significance Threshold" at Mean + 5.5*StdDev
    plt.axhline(y=mean_val + (6 * std_val), color='r', linestyle='--', label='6σ Threshold')
    #plt.axhline(y=5, color='g', linestyle='--', label='3 Stars Threshold')

    plt.title(f'Peak Detection (Z-Score: {z_score:.2f})')
    plt.legend()
    plt.savefig('correlation_analysis.jpg')
    """
  #True if Z-score uis above threshold and at least N stars were found
  stars_matched = (z_score > threshold)
  print("Z-score: ", z_score,"Matched: ", stars_matched)

  return stars_matched, z_score, max_matched_angle

def findPolaris(star_candidates, stars, database_vectors, threshold, radius_arcmin, star_count, debug = False): #Radius is in arcmin default 107.5arcmin (~506px)
  for star in star_candidates:
      radius_stars = getStarsInRadius(star, stars, radius_arcmin * defaultPixPerArcmin)

      #Check if too many stars were found near polaris and exit in case it did
      if (len(radius_stars) > 50):
        print("Too many stars in radius, probably picked up some noise -> ABORT")
        return False, None, None

      print("Number of stars in radius:", len(radius_stars))
      star_vectors = star - radius_stars[:star_count]
      polarisFound,_,angle = starCorrelation(star_vectors, database_vectors, threshold, 0.25, debug)
      if polarisFound == True:
        return True, star, angle

  return False, None, None

def findNCPStars(polaris, stars, database_ncp_vectors, central_angle, radius_arcmin, star_count, length_tolerance=0.05, debug=False):
  
  angle_step = 0.25
  cos_threshold = 0.99996 #cos(0.25°) = 0.99999048
  
  #Get stars in a radius around Poalris
  radius_stars = getStarsInRadius(polaris, stars, radius_arcmin * defaultPixPerArcmin)
  #Select brightest n brightests stars
  star_vectors = polaris - radius_stars[:star_count]

  
  yildun_vector = database_ncp_vectors[0]
  ov_cephei_vector = database_ncp_vectors[1]
  ursae_minoris_2_vector = database_ncp_vectors[2]

  yildun_vector_unit = (yildun_vector / np.linalg.norm(yildun_vector)).astype(np.float32)
  yildun_length = np.linalg.norm(yildun_vector)

  ov_cephei_vector_unit = (ov_cephei_vector / np.linalg.norm(ov_cephei_vector)).astype(np.float32)
  ov_cephei_length = np.linalg.norm(ov_cephei_vector)

  ursae_minoris_2_vector_unit = (ursae_minoris_2_vector/ np.linalg.norm(ursae_minoris_2_vector)).astype(np.float32)
  ursae_minoris_2_length = np.linalg.norm(ursae_minoris_2_vector)

  best_match_count = 0

  star_vectors_unit = (star_vectors / np.linalg.norm(star_vectors, axis=1)[:, np.newaxis]).astype(np.float32)
  star_lengths = np.linalg.norm(star_vectors, axis=1)

  angles = np.arange(central_angle-5, central_angle+5, angle_step)

  best_angle        = central_angle
  best_match_count  = 0
  yildun_idx        = None
  ov_cephei_idx     = None
  ursae_minoris_2_idx = None

  for angle in angles:
    angle_radians = np.radians(angle)

    rotation_matrix = np.array([[np.cos(angle_radians), -np.sin(angle_radians)],
                                [np.sin(angle_radians),  np.cos(angle_radians)]])

    rotated_stars = star_vectors_unit @ rotation_matrix.T

    yildun_dot = np.dot(rotated_stars, yildun_vector_unit)
    ov_cephei_dot = np.dot(rotated_stars, ov_cephei_vector_unit)
    ursae_minoris_2_dot = np.dot(rotated_stars, ursae_minoris_2_vector_unit)

    # Find heading-matched star indices for each db star
    yildun_candidates = np.where(yildun_dot > cos_threshold)[0]
    ov_cephei_candidates = np.where(ov_cephei_dot > cos_threshold)[0]
    ursae_minoris_2_candidates = np.where(ursae_minoris_2_dot > cos_threshold)[0]

    # Length check for each candidate
    yildun_result = None
    for i in yildun_candidates:
        ratio = star_lengths[i] / yildun_length
        if abs(ratio - 1.0) <= length_tolerance:
            yildun_result = i
            break

    ov_cephei_result = None
    for i in ov_cephei_candidates:
        ratio = star_lengths[i] / ov_cephei_length
        if abs(ratio - 1.0) <= length_tolerance:
            ov_cephei_result = i
            break

    ursae_minoris_2_result = None
    for i in ursae_minoris_2_candidates:
        ratio = star_lengths[i] / ursae_minoris_2_length
        if abs(ratio - 1.0) <= length_tolerance:
            ursae_minoris_2_result = i
            break

    match_count = sum(1 for r in [yildun_result, ov_cephei_result, ursae_minoris_2_result] if r is not None)

    if debug:
        print(f"Angle:   {angle:.2f}°    Yildun:  {yildun_result}     OV Cephei: {ov_cephei_result}     2 Ursae Minoris: {ursae_minoris_2_result}")

    if match_count > best_match_count:
        best_match_count    = match_count
        best_angle          = angle
        yildun_idx          = yildun_result
        ov_cephei_idx       = ov_cephei_result
        ursae_minoris_2_idx = ursae_minoris_2_result

  if debug:
      print(f"Best angle: {best_angle:.2f}°  Matches: {best_match_count}/3")
      print(f"Yildun: {yildun_idx}  OV Cephei: {ov_cephei_idx}  2 Ursae Minoris: {ursae_minoris_2_idx}")

  if (yildun_idx is not None):
    yildun = radius_stars[yildun_idx]
  else:
    yildun = None

  if (ov_cephei_idx is not None):
    ov_cephei = radius_stars[ov_cephei_idx]
  else:
    ov_cephei = None

  if (ursae_minoris_2_idx is not None):
    ursae_minoris_2 = radius_stars[ursae_minoris_2_idx]
  else:
    ursae_minoris_2 = None

  return yildun, ov_cephei, ursae_minoris_2

def trilaterateNCP(starA_coordinates, starB_coordinates, starC_coordinates, starA_NCP_dist, starB_NCP_dist, starC_NCP_dist):
    ex = (starB_coordinates - starA_coordinates)
    ex = ex / np.linalg.norm(ex)
    i  = np.dot(ex, starC_coordinates - starA_coordinates)
    ey = starC_coordinates - starA_coordinates - i * ex
    ey = ey / np.linalg.norm(ey)
    d  = np.linalg.norm(starB_coordinates - starA_coordinates)
    j  = np.dot(ey, starC_coordinates - starA_coordinates)

    # Compute coordinates
    x = (starA_NCP_dist**2 - starB_NCP_dist**2 + d**2) / (2 * d)
    y = (starA_NCP_dist**2 - starC_NCP_dist**2 + i**2 + j**2 - 2*i*x) / (2 * j)

    # Calculate final NCP position
    NCP_coordinates = starA_coordinates + x * ex + y * ey
    return NCP_coordinates

def getAngleToZeroRA(Polaris, Star, RAangle):
  angle = np.arctan2((Star[1] - Polaris[1]), Star[0] - Polaris[0])

  angle = np.degrees(angle) - RAangle
  if(angle < 0):
    angle = 360 + angle

  return angle

def getNCPposition(polaris_coordinates, yildun_coordinates, ov_cephei_coordinates, ursae_minoris_2_coordinates, image_center, star_distances, star_Polaris_RA_angle):
    # Separation distances in arcmin (P=Polaris, Y=Yildun, C=OV Cephei, U=2 Ursae Minoris, N=NCP)
    PY_separation, PC_separation, PU_separation, PN_separation, YC_separation, YU_separation, YN_separation, CU_separation, CN_separation, UN_separation = star_distances

    #Bundle all detected stars, at least 2 stars and Polaris are needed
    available_stars = []
    if yildun_coordinates is not None:
        available_stars.append(("Yildun", yildun_coordinates, PY_separation, YN_separation, star_Polaris_RA_angle[0]))
    if ov_cephei_coordinates is not None:
        available_stars.append(("OV Cephei", ov_cephei_coordinates, PC_separation, CN_separation, star_Polaris_RA_angle[1]))
    if ursae_minoris_2_coordinates is not None:
        available_stars.append(("2 Ursae Minoris",  ursae_minoris_2_coordinates, PU_separation, UN_separation, star_Polaris_RA_angle[2])) #P, Y, OV, UM - 3.08127778 17.39761111  7.86916667  1.20791667

    if len(available_stars) < 2:
        print("Not enough stars found for trilateration")
        return False, None, None, None

    #Average pixels per arcmin
    pix_per_arcmin_samples = []
    for name, coords, P_separation, _ , _ in available_stars:
        dist_px = np.linalg.norm(polaris_coordinates - coords)
        pix_per_arcmin_samples.append(dist_px / P_separation)
        print(f"Pixels per arcmin Polaris-{name}: {dist_px / P_separation:.2f}")

    averagePixPerArcmin = np.mean(pix_per_arcmin_samples)
    print(f"Average pixels per arcmin: {averagePixPerArcmin:.2f}")


    #Trilaterate using Polaris and two available stars
    starA_coordinates, starA_NCP_dist = polaris_coordinates, averagePixPerArcmin * PN_separation
    starB_coordinates, starB_NCP_dist = available_stars[0][1], averagePixPerArcmin * available_stars[0][3]
    starC_coordinates, starC_NCP_dist = available_stars[1][1], averagePixPerArcmin * available_stars[1][3]

    print(f"\nTrilateration using Polaris + {available_stars[0][0]} + {available_stars[1][0]}")

    NCP_coordinates = trilaterateNCP(starA_coordinates, starB_coordinates, starC_coordinates,
                                     starA_NCP_dist,    starB_NCP_dist,    starC_NCP_dist)

    print("NCP error pixels: ", image_center - NCP_coordinates)

    NCP_error = (image_center - NCP_coordinates) / averagePixPerArcmin
    print(f"NCP error: {NCP_error[0]:.2f}, {NCP_error[1]:.2f} arcmin")

    #Calculate 0-hour RA angle
    sin_sum = 0
    cos_sum = 0

    for _ , star_coordinates, _, _, RA_angle in available_stars:
      RAangle = getAngleToZeroRA(polaris_coordinates, star_coordinates, RA_angle)

      sin_sum = sin_sum + np.sin(np.radians(RAangle))
      cos_sum = cos_sum + np.cos(np.radians(RAangle))

    zeroRAangle = np.degrees(np.arctan2(sin_sum/len(available_stars), cos_sum/len(available_stars)))
    
    print(f"Zero RA angle: {zeroRAangle:.5f}°")
    
    return True, NCP_coordinates, NCP_error, zeroRAangle


def drawNCP(image, ncp_coordinates, polaris_coordinates, yildun_coordinates, ov_cephei_coordinates, ursae_minoris_2_coordinates, zero_RA_angle):
    
    output = image.copy()

    # --- Helper functions ---
    def unit_vector(from_pt, to_pt):
        vec = to_pt - from_pt
        return vec / np.linalg.norm(vec)

    def rotate_vector(vec, angle_deg):
        angle_rad = np.radians(angle_deg)
        rotation_matrix = np.array([
            [np.cos(angle_rad), -np.sin(angle_rad)],
            [np.sin(angle_rad),  np.cos(angle_rad)]
        ])
        return rotation_matrix @ vec

    def draw_vector_line(origin, direction_vec, length=1250, color=(0, 255, 0), thickness=1):
        end_point = origin + direction_vec * length
        cv2.line(output, tuple(origin.astype(int)), tuple(end_point.astype(int)), color, thickness)

    def mark_star(coordinates, label):
        cv2.circle(output, tuple(np.array(coordinates).astype(int)), 15, (0, 255, 0), 2)
        cv2.putText(output, label,
                    (int(coordinates[0]) + 13, int(coordinates[1]) - 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)

    # --- Mark NCP ---
    cv2.circle(output, tuple(ncp_coordinates.astype(int)), 5, (0, 255, 0), 2)
    cv2.putText(output, "NCP",
                (int(ncp_coordinates[0]) + 13, int(ncp_coordinates[1]) - 13),
                cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)

    # --- Mark all stars (skip None) ---
    if polaris_coordinates is not None:
        mark_star(polaris_coordinates,        "Polaris")
    if yildun_coordinates is not None:
        mark_star(yildun_coordinates,         "Yildun")
    if ov_cephei_coordinates is not None:
        mark_star(ov_cephei_coordinates,      "OV Cephei")
    if ursae_minoris_2_coordinates is not None:
        mark_star(ursae_minoris_2_coordinates,"2 Ursae Minoris")

    # --- Draw RA hour lines from zero_RA_angle ---
    zero_RA_angle_rad = np.radians(zero_RA_angle)
    zero_RA_direction = np.array([np.cos(zero_RA_angle_rad), np.sin(zero_RA_angle_rad)])

    RA_lines = [
        (  0, zero_RA_direction,                    (255,   0,   0), "0h" ),
        ( 90, rotate_vector(zero_RA_direction,  90), (  0, 255,   0), "6h" ),
        (180, rotate_vector(zero_RA_direction, 180), (  0,   0, 255), "12h"),
        (270, rotate_vector(zero_RA_direction, 270), (255, 255,   0), "18h"),
    ]

    for _, direction, color, label in RA_lines:
        draw_vector_line(ncp_coordinates, direction, length=1250, color=color, thickness=1)
        label_position = ncp_coordinates + direction * 1280
        cv2.putText(output, label,
                    tuple(label_position.astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, color, 3)

    cv2.imwrite('ncp.jpg', output)
    return output


def getCenterOfRotation(image_zero, image_angled):
  if image_zero is None or image_angled is None:
    return None
  
  preprocessed_image_zero = imagePreprocessing(image_zero)
  preprocessed_image_angled = imagePreprocessing(image_angled)

  stars_zero, brightest_stars_zero = findStars(preprocessed_image_zero, number_of_brightest_stars = 10)
  stars_angled, brightest_stars_angled = findStars(preprocessed_image_angled, number_of_brightest_stars = 10)

  polarisFound_zero, Polaris_zero, Angle_zero = findPolaris(brightest_stars_zero, stars_zero, polaris_database_vectors, threshold=6.0, radius_arcmin=107.5, star_count=25, debug=True)
  polarisFound_angled, Polaris_angled, Angle_angled = findPolaris(brightest_stars_angled, stars_angled, polaris_database_vectors, threshold=6.0, radius_arcmin=107.5, star_count=25, debug=True)

  if polarisFound_zero == False or polarisFound_angled == False:
    print("ERROR: Polaris not found!")
    return None

  Yildun_zero, OV_Cephei_zero, Ursae_Minoris_2_zero = findNCPStars(Polaris_zero, stars_zero, ncp_database_vectors, Angle_zero, radius_arcmin=250, star_count=10, length_tolerance=0.05, debug=True)
  Yildun_angled, OV_Cephei_angled, Ursae_Minoris_2_angled = findNCPStars(Polaris_angled, stars_angled, ncp_database_vectors, Angle_angled, radius_arcmin=250, star_count=10, length_tolerance=0.05, debug=True)

  pairs_zero = []
  pairs_angled = []

  stars_to_check = [
    (Polaris_zero, Polaris_angled),
    (Yildun_zero, Yildun_angled),
    (OV_Cephei_zero, OV_Cephei_angled),
    (Ursae_Minoris_2_zero, Ursae_Minoris_2_angled)
  ]

  for star_zero, star_angled in stars_to_check:
    if star_zero is not None and star_angled is not None:
      pairs_zero.append([star_zero[0], star_zero[1]])
      pairs_angled.append([star_angled[0], star_angled[1]])

  #numpy arrays for the calculation
  stars = np.array(pairs_zero)
  stars_prime = np.array(pairs_angled)

  center = calculateCoR(stars, stars_prime)
  if (center is None):
    return None
    
  center_offset = center-(np.array([image_zero.shape[1] // 2, image_zero.shape[0] // 2]))

  return center_offset

def calculateCoR(P, P_prime):
    centroid_P = np.mean(P, axis=0)
    centroid_P_prime = np.mean(P_prime, axis=0)
    
    P_centered = P - centroid_P
    P_prime_centered = P_prime - centroid_P_prime

    #singular value decomposition to find optimal Rot matrix
    #finds the rotation that best maps P to P_prime
    H = P_centered.T @ P_prime_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[1,:] *= -1
        R = Vt.T @ U.T

    #P' = R(P - C) + C -> (I - R)C = P' - RP
    I = np.eye(2)
    t = centroid_P_prime - (R @ centroid_P)
    
    try:
        center = np.linalg.solve(I - R, t)
        
        #Print the actual rotation angle
        actual_angle = np.degrees(np.arctan2(R[1,0], R[0,0]))
        print(f"Detected rotation: {actual_angle:.4f} degrees")
        
        return center
    except np.linalg.LinAlgError:
        return None

def getAlignmentError(image, center_offset, debug=False):
  #Check if the image was loaded successfully
  if image is None:
      print("ERROR: Image not loaded")
      #Error should be sent to PCK main board here
      return 0,0,False,False,0

  preprocessed_image = imagePreprocessing(image)
  stars, brightest_stars = findStars(preprocessed_image, number_of_brightest_stars = 10)

  polarisFound, Polaris, Angle = findPolaris(brightest_stars, stars, polaris_database_vectors, threshold=6.0, radius_arcmin=107.5, star_count=25, debug=True)

  if polarisFound == False:
    print("ERROR: Polaris not found!")
    return 0,0,False,False,0

  Yildun, OV_Cephei, Ursae_Minoris_2 = findNCPStars(Polaris, stars, ncp_database_vectors, Angle, radius_arcmin=250, star_count=10, length_tolerance=0.05, debug=True)

  #Compute the pixel position of the center of rotation
  image_center = np.array([image.shape[1] // 2, image.shape[0] // 2])

  if center_offset is None:
    center_of_rotation = image_center
  else:
    center_of_rotation = center_offset + image_center

  ncpFound, NCP_coordinates, NCP_error, zero_RA_angle = getNCPposition(Polaris, Yildun, OV_Cephei, Ursae_Minoris_2, center_of_rotation, star_distances, star_Polaris_RA_angle)

  if (debug and ncpFound):
    drawNCP(image, NCP_coordinates, Polaris, Yildun, OV_Cephei, Ursae_Minoris_2, zero_RA_angle)

    print("Polaris", Polaris, "NCP", NCP_coordinates, "Yildun", Yildun, "OV Cephein", OV_Cephei,"2 Ursae Minoris", Ursae_Minoris_2)

  #Both Polaris and NCP found
  if (polarisFound == True and ncpFound == True):
    RA_angle = (360.0 - zero_RA_angle) % 360.0
    return NCP_error[0], NCP_error[1], polarisFound, ncpFound, RA_angle
  #Polaris found but finding NCP unsuccesful
  elif (polarisFound == True and ncpFound == False):
    Polaris_error = (center_of_rotation-Polaris) / defaultPixPerArcmin
    return Polaris_error[0],Polaris_error[1],polarisFound,ncpFound, 0
  else:
    return 0,0,False,False,0