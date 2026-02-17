import cv2
import numpy as np
#from google.colab.patches import cv2_imshow
import math
#import matplotlib.pyplot as plt

polaris_vectors = np.load("star_vectors/polaris_vectors.npy")
yildun_vectors = np.load("star_vectors/yildun_vectors.npy")
ov_cephei_vectors = np.load("star_vectors/ov_cephei_vectors.npy")

default_pix_per_arcmin = 4608/(14.334*60) #Default pixels per arminute - Later load from config file

stars_to_find = 35
stars_threshold = 28
stars_to_loop_through = 10

def image_preprocessing(image):
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

  #Get mean pixel value from the image
  mean = np.mean(gray)

  #Increase the image contrast based on mean pixel value
  contrast_img = cv2.convertScaleAbs(gray, alpha=1, beta=-mean)
  
  # Apply Gaussian blur to reduce noise
  blurred = cv2.GaussianBlur(contrast_img, (5, 5), 0)
  
  # Get threshold from contrasted and blurred image
  thresh, _ = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
  
  # Limit the threshold
  min_thresh = 25
  max_thresh = 100
  threshold = max(min_thresh, min(thresh, max_thresh))
  #print(threshold)

  # Apply adaptive thresholding to detect bright spots
  _, thresholded = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY) #If pix value < threshold  --->  turns pixel black

  cv2.imwrite('thresholded.jpg', thresholded)

  return thresholded

def brightest_stars(number_of_stars, image):
  # Find contours (potential stars)
  contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  # Filter out small contours based on area (to avoid noise)
  min_area = 1  # Minimum area for contours to be considered as stars
  contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
  # Sort contours by area in descending order (for selecting biggest stars)
  contours = sorted(contours, key=cv2.contourArea, reverse=True)
  # Select only the top X largest contours (stars)
  top_contours = contours[:number_of_stars]
  star_centers = []
  for i, cnt in enumerate(top_contours):
      # Get the center and radius of the circle for each contour
      (x, y), _ = cv2.minEnclosingCircle(cnt)
      center = (int(x), int(y))
      #radius = int(radius)
      star_centers.append([int(x),int(y)])
  star_centers = np.array(star_centers)
  return star_centers


def find_polaris(center_star, surrounding_stars, matched_vectors_threshold, star_pattern):
    star_pattern = star_pattern.astype(np.float32)
    
    #Compute vectors from center star to surrounding stars
    vectors_to_center = (center_star - surrounding_stars).astype(np.float32) 
    
    #Normalizing
    norms = np.linalg.norm(vectors_to_center, axis=1)[:, np.newaxis]
    normalized_vectors = vectors_to_center / (norms + 1e-9)

    max_matched_vectors = 0
    max_matched_angle = 0

    matching_angles = []

    #Dot product threshold
    cos_threshold = 0.999 #cos(+-2.5°) = 0.999048...

    for angle in np.arange(-180, 180, 2.5):
        angle_radians = np.radians(angle)
        
        # 4. Rotation Matrix (Standard clockwise/counter-clockwise check)
        rotation_matrix = np.array([[np.cos(angle_radians), -np.sin(angle_radians)],
                                  [np.sin(angle_radians), np.cos(angle_radians)]])
        
        # Rotate image vectors
        rotated_vectors = np.dot(normalized_vectors, rotation_matrix)

        # Compare against database
        dot_matrix = np.dot(rotated_vectors, star_pattern.T)

        # Find best database match for each image star
        best_matches_per_vector = np.max(dot_matrix, axis=1)
        num_matching_pairs = np.count_nonzero(best_matches_per_vector > cos_threshold)

        if num_matching_pairs >= stars_threshold:
          matching_angles.append(angle)
          print("Angle:", angle, "Matching Pairs:", num_matching_pairs)

          if num_matching_pairs > max_matched_vectors:
            max_matched_vectors = num_matching_pairs
            max_matched_angle = angle

    #print("Max number of matching vectors:", max_matched_vectors)
    #print("Angle with the maximum number of matching vectors:", max_macthed_angle)
    if max_matched_vectors >= matched_vectors_threshold:
      PolarisFound = check_star(center_star, surrounding_stars, matched_vectors_threshold, star_pattern, max_matched_angle)
      return PolarisFound, max_matched_angle
    else:
      return False, None

def check_star(center_star, surrounding_stars, matched_vectors_threshold, star_pattern, img_angle):
  #vectors_to_center = center_star - surrounding_stars
  #normalized_vectors = vectors_to_center / np.linalg.norm(vectors_to_center, axis=1)[:, np.newaxis]

  star_pattern = star_pattern.astype(np.float32)
    
  #Compute vectors from center star to surrounding stars
  vectors_to_center = (center_star - surrounding_stars).astype(np.float32) 
    
  #Normalizing
  norms = np.linalg.norm(vectors_to_center, axis=1)[:, np.newaxis]
  normalized_vectors = vectors_to_center / (norms + 1e-9)

  matching_angles = []
  max_matched_vectors = 0
  max_matched_angle = 0

  cos_threshold = 0.99999

  for angle in np.arange(img_angle-5, img_angle+5, 0.25):
      angle_radians = np.radians(angle)
      rotation_matrix = np.array([[np.cos(angle_radians), -np.sin(angle_radians)],
                                  [np.sin(angle_radians), np.cos(angle_radians)]])
      rotated_vectors = np.dot(normalized_vectors, rotation_matrix)

      # Find matching pairs using broadcasting and isclose
      # Compare against database
      dot_matrix = np.dot(rotated_vectors, star_pattern.T)

      # Find best database match for each image star
      best_matches_per_vector = np.max(dot_matrix, axis=1)
      num_matching_pairs = np.count_nonzero(best_matches_per_vector > cos_threshold)

      if num_matching_pairs >= stars_threshold:
        matching_angles.append(angle)
        print("Angle:", angle, "Matching Pairs:", num_matching_pairs)

        if num_matching_pairs > max_matched_vectors:
          max_matched_vectors = num_matching_pairs
          max_matched_angle = angle

    #print("Max number of matching vectors:", max_macthed_vectors)
    #print("Angle with the maximum number of matching vectors:", max_macthed_angle)
  if max_matched_vectors >= matched_vectors_threshold:
    return True
  else:
    return False

def trilaterate_ncp(polaris_coordinates, yildun_coordinates, ov_cephei_coordinates, polaris_ncp_dis, yildun_ncp_dis, ov_cephei_ncp_dis):
  ex = (yildun_coordinates - polaris_coordinates)
  ex = ex / np.linalg.norm(ex)
  i = np.dot(ex, ov_cephei_coordinates - polaris_coordinates)
  ey = ov_cephei_coordinates - polaris_coordinates - i * ex
  ey = ey / np.linalg.norm(ey)
  d = np.linalg.norm(yildun_coordinates - polaris_coordinates)
  j = np.dot(ey, ov_cephei_coordinates - polaris_coordinates)

  # Compute coordinates
  x = (polaris_ncp_dis**2 - yildun_ncp_dis**2 + d**2) / (2 * d)
  y = (polaris_ncp_dis**2 - ov_cephei_ncp_dis**2 + i**2 + j**2 - 2*i*x) / (2 * j)

  # Calculate final position
  ncp_coordinates = polaris_coordinates + x * ex + y * ey
  return ncp_coordinates


def get_ncp_position(polaris_coordinates, yildun_coordinates, ov_cephei_coordinates, polaris_yildun_dis, polaris_ov_cephei_dis, yildun_ov_cephei_dis, image):
  #Separation distances in arcmin (Polaris = P, Yildun = Y, North = N, OV Cephei = C)
  PY_separation = 238.38
  PC_separation = 173.93
  PN_separation = 37.82
  YC_separation = 368.75
  YN_separation = 206.37
  CN_separation = 182.13

  avg_pix_per_arcmin = (polaris_yildun_dis/PY_separation + polaris_ov_cephei_dis/PC_separation + yildun_ov_cephei_dis/YC_separation) / 3

  #print("Average pixels per arcmin:",avg_pix_per_arcmin)
  #print("Pixels per arcmin:", (polaris_yildun_dis/PY_separation))
  #print("Pixels per arcmin:", (polaris_ov_cephei_dis/PC_separation))
  #print("Pixels per arcmin:", (yildun_ov_cephei_dis/YC_separation))

  polaris_ncp_dis = avg_pix_per_arcmin * PN_separation
  yildun_ncp_dis = avg_pix_per_arcmin * YN_separation
  ov_cephei_ncp_dis = avg_pix_per_arcmin * CN_separation

  print("\nPolaris NCP distance in pixels:", f"{polaris_ncp_dis:.2f}")
  print("Yildun NCP distance in pixels:", f"{yildun_ncp_dis:.2f}")
  print("OV Cephei NCP distance in pixels:", f"{ov_cephei_ncp_dis:.2f}")

  ncp_coordinates = trilaterate_ncp(polaris_coordinates, yildun_coordinates, ov_cephei_coordinates, polaris_ncp_dis, yildun_ncp_dis, ov_cephei_ncp_dis)

  image_center = np.array([image.shape[1] // 2, image.shape[0] // 2])
  ncp_error = (image_center - ncp_coordinates)/avg_pix_per_arcmin
  return ncp_coordinates ,ncp_error

def unit_vector(from_pt, to_pt):
    vec = to_pt - from_pt
    return vec / np.linalg.norm(vec)

def rotate_vector(vec, angle_deg):
    angle_rad = np.radians(angle_deg)
    rot_matrix = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad),  np.cos(angle_rad)]
    ])
    return rot_matrix @ vec

def draw_vector_line(image, origin, vec, length=1250, color=(0,255,0), thickness=1):
    end_pt = origin + vec * length
    cv2.line(image, tuple(origin.astype(int)), tuple(end_pt.astype(int)), color, thickness)

def draw_ncp(ncp_coordinates, polaris_coordinates, yildun_coordinates, ov_cephei_coordinates, ncp_output):
  #ncp_output = image.copy()
  cv2.circle(ncp_output, (int(ncp_coordinates[0]), int(ncp_coordinates[1])), 5, (0, 255, 0), 2)
  cv2.putText(ncp_output, "NCP", (int(ncp_coordinates[0]) + 13, int(ncp_coordinates[1]) - 13), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)

  cv2.circle(ncp_output, polaris_coordinates, 5, (0, 255, 0), 2)
  cv2.circle(ncp_output, yildun_coordinates, 5, (0, 255, 0), 2)
  cv2.circle(ncp_output, ov_cephei_coordinates, 5, (0, 255, 0), 2)

  #Drawing hour lines

  v_polaris = unit_vector(ncp_coordinates, polaris_coordinates)
  v_yildun = unit_vector(ncp_coordinates, yildun_coordinates)
  v_ovcephei = unit_vector(ncp_coordinates, ov_cephei_coordinates)

  ras_deg = {
    'polaris': 3.0835 * 15,
    'yildun': 17.4016 * 15,
    'ovcephei': 7.8525 * 15
  }

  vectors = {
      ras_deg['polaris']: v_polaris,
      ras_deg['yildun']: v_yildun,
      ras_deg['ovcephei']: v_ovcephei
  }

  zero_ra_dir = np.zeros(2)

  for ra_deg, vec in vectors.items():
      zero_ra_dir += rotate_vector(vec, -ra_deg)

  zero_ra_dir /= np.linalg.norm(zero_ra_dir)

  v_6h  = rotate_vector(zero_ra_dir, 90)
  v_12h = rotate_vector(zero_ra_dir, 180)
  v_18h = rotate_vector(zero_ra_dir, 270)

  draw_vector_line(ncp_output, ncp_coordinates, zero_ra_dir, color=(255,0,0))    # 0h
  draw_vector_line(ncp_output, ncp_coordinates, v_6h,        color=(0,255,0))    # 6h
  draw_vector_line(ncp_output, ncp_coordinates, v_12h,       color=(0,0,255))    # 12h
  draw_vector_line(ncp_output, ncp_coordinates, v_18h,       color=(255,255,0))  # 18h

  cv2.imwrite('ncp_output.jpg', ncp_output)


def getAlignmentError(image):
  #Loads sets of star vectors

  #Check if the image was loaded successfully
  if image is None:
      print("Error: Could not load image. Please check the file path.")
      #Error should be sent to PCK main board here
      return 0,0,False,False
  
  #Preprocess the image
  preprocessed_image = image_preprocessing(image)
  #Gets centers of brightest stars
  star_centers = brightest_stars(stars_to_find, preprocessed_image) #Number of stars to be found, image
  #print(star_centers.shape) #just for debugging to see how many stars were found
  stars_found = len(star_centers) if star_centers is not None else 0
  if (stars_found != stars_to_find): #Return if the number of stars is lower than the required amount
    return 0,0,False,False

  polaris_coordinates = np.array([])
  yildun_coordinates = np.array([])
  ov_cephei_coordinates = np.array([])

  for index, star in enumerate(star_centers[:stars_to_loop_through]):
    polarisFound, img_angle = find_polaris(star, np.append(star_centers[:index],star_centers[(index+1):], axis=0), stars_threshold, polaris_vectors) #Check if current star is Polaris and at which angle the Polaris was found compared to the vector set
    if polarisFound == True:
      print("Polaris is in the image at: X =", star[0], "Y =", star[1])
      polaris_coordinates = star
      break
  print("IMG is at angle:", img_angle)

  if polaris_coordinates.size != 0:
    for index, star in enumerate(star_centers[:stars_to_loop_through]):
      if check_star(star, np.append(star_centers[:index],star_centers[(index+1):], axis=0), stars_threshold, yildun_vectors, img_angle): #Checks if current star is Yildun
        print("Yildun is in the image at: X =", star[0], "Y =", star[1])
        yildun_coordinates = star
        break

    for index, star in enumerate(star_centers[:stars_to_loop_through]):
      #print(index)
      #print(star_centers[:index])
      if check_star(star, np.append(star_centers[:index],star_centers[(index+1):], axis=0), stars_threshold, ov_cephei_vectors, img_angle): #Checks if current star is OV Cephei
        print("OV Cephei is in the image at: X =", star[0], "Y =", star[1])
        ov_cephei_coordinates = star
        break
  else:
    print("Polaris not found")
    polarisFound = False
    #Error should be sent to PCK main board

  ncp_error = np.array([0,0])

  #Distance between Polaris and Yildun in pixels
  if polaris_coordinates.size != 0 and yildun_coordinates.size != 0 and ov_cephei_coordinates.size !=0: #Checks if all the stars were found
    polaris_yildun_dis = np.linalg.norm(polaris_coordinates - yildun_coordinates)
    polaris_ov_cephei_dis = np.linalg.norm(polaris_coordinates - ov_cephei_coordinates)
    yildun_ov_cephei_dis = np.linalg.norm(yildun_coordinates - ov_cephei_coordinates)

    allStarsFound = True
    #print("\nPolaris Yildun distance in pixels:", f"{polaris_yildun_dis:.2f}")
    #print("Polaris OV Cephei distance in pixels:", f"{polaris_ov_cephei_dis:.2f}")
    #print("Yildun OV Cephei distance in pixels:", f"{yildun_ov_cephei_dis:.2f}")

    ncp_coordinates, ncp_error = get_ncp_position(polaris_coordinates, yildun_coordinates, ov_cephei_coordinates, polaris_yildun_dis, polaris_ov_cephei_dis, yildun_ov_cephei_dis, image)

    print("\nNCP coordinates:", ncp_coordinates)
    print("NCP error from img center in arcmin:", ncp_error)

    draw_ncp(ncp_coordinates, polaris_coordinates, yildun_coordinates, ov_cephei_coordinates, image)

  elif polaris_coordinates.size !=0 and (yildun_coordinates.size == 0 or ov_cephei_coordinates.size == 0): #Polaris found but cannot trilaterate - insufficent stars found
    image_center = np.array([image.shape[1] // 2, image.shape[0] // 2])
    ncp_error = (image_center - polaris_coordinates)/default_pix_per_arcmin
    allStarsFound = False

  else:
    print("Polaris or other stars not found")
    allStarsFound = False

  #Function should return polarisFound (True/False), allStarsFound (True,False), NCP error (arcmin) - [0,0] if not found
  return ncp_error[0], ncp_error[1], polarisFound, allStarsFound

#image = cv2.imread("/home/pck/pckdev/star_img/polaris5.jpg")
#NCPErrorX, NCPErrorY, PF, ASF = polarAlign(image)
#polaris_img = cv2.imread("/content/polaris5.jpg")
#generate_vectors(polaris_img, 100)
#print("Polaris found:",PF,"   All stars found: ",ASF)
#print("NCP X Error in arcmin is:", NCPErrorX)
#print("NCP Y Error in arcmin is:", NCPErrorY)