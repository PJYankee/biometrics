import os
import json

from PIL import Image, ImageDraw

if __name__ == "__main__":

    '''
    for filename in os.listdir(os.getcwd() + "/LeftJSON"):
        with open(os.path.join("LeftJSON", filename), 'r') as f:
            skeleton = json.load(f)

            maxSkeletonLength = 0
            maxSkeletonIndex = 0
            
            for index, person in enumerate(skeleton['people']):
                
                skeletonLength = 0
                
                for keyPoint in person['pose_keypoints_2d']:
                    
                    if keyPoint > 0:
                        skeletonLength += 1

                if skeletonLength > maxSkeletonLength:
                    maxSkeletonLength = skeletonLength
                    maxSkeletonIndex = index
                    
            outputJSON = json.dumps(skeleton['people'][maxSkeletonIndex]['pose_keypoints_2d'])

            with open(os.getcwd() + '/ProcessedLeft/' + filename.split('_')[0] + '_processed.json', "w") as outfile:
                outfile.write(outputJSON)

            with Image.open(os.getcwd() + "/LeftImages/" + filename.split('_')[0] + "_rendered.png") as im:
                draw = ImageDraw.Draw(im)

                xCord = skeleton['people'][maxSkeletonIndex]['pose_keypoints_2d'][0]
                yCord = skeleton['people'][maxSkeletonIndex]['pose_keypoints_2d'][1]

                draw.text((xCord,yCord), "SKELETON", fill = (255, 0, 0))

                im.save(os.getcwd() + '/ProcessedLeftImages/' + filename.split('_')[0] + '_processed.png')

            
    '''

    for filename in os.listdir(os.getcwd() + "/RightJSON"):
        with open(os.path.join("RightJSON", filename), 'r') as f:
            skeleton = json.load(f)

            maxSkeletonLength = 0
            maxSkeletonIndex = 0
            
            for index, person in enumerate(skeleton['people']):
                
                skeletonLength = 0
                
                for keyPoint in person['pose_keypoints_2d']:
                    
                    if keyPoint > 0:
                        skeletonLength += 1

                if skeletonLength > maxSkeletonLength:
                    maxSkeletonLength = skeletonLength
                    maxSkeletonIndex = index
                    
            outputJSON = json.dumps(skeleton['people'][maxSkeletonIndex]['pose_keypoints_2d'])

            with open(os.getcwd() + '/ProcessedRight/' + filename.split('_')[0] + '_processed.json', "w") as outfile:
                outfile.write(outputJSON)

            with Image.open(os.getcwd() + "/RightImages/" + filename.split('_')[0] + "_rendered.png") as im:
                draw = ImageDraw.Draw(im)

                xCord = skeleton['people'][maxSkeletonIndex]['pose_keypoints_2d'][0]
                yCord = skeleton['people'][maxSkeletonIndex]['pose_keypoints_2d'][1]

                draw.text((xCord,yCord), "SKELETON", fill = (255, 0, 0))

                im.save(os.getcwd() + '/ProcessedRightImages/' + filename.split('_')[0] + '_processed.png')

            
