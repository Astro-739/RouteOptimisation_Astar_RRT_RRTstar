import random
from icecream import ic
from Astar_utils import RiskZone


def make_circle_riskzones(mapdimensions:tuple[int],
                          riskzone_radius:tuple[int],
                          num_riskzones:int
                          ) -> list[RiskZone]:
    # init
    riskzones = []
    # make list of obstacles that can cover start or goal
    for _ in range(num_riskzones):
        circle = make_random_circle(mapdimensions, riskzone_radius)
        riskzone = RiskZone((circle[0],circle[1]),circle[2])
        riskzones.append(riskzone)
    # return result
    return riskzones


def read_circle_riskzones() -> list[RiskZone]:
    # issues with startnode
    dict1:dict = {(698, 171): 54, (456, 717): 195, (900, 813): 114, (214, 880): 196, (774, 131): 187, (406, 18): 152, (903, 39): 160, (514, 214): 151, (165, 838): 120, (162, 189): 133, (759, 843): 139, (952, 386): 98}
    dict2:dict = {(192, 51): 199, (609, 738): 104, (555, 791): 139, (661, 663): 162, (672, 78): 127, (582, 768): 179, (951, 588): 120, (736, 740): 94, (161, 346): 162, (839, 757): 184, (266, 158): 63, (685, 565): 75}
    # node = LOSbasenode issue (11 also weird node inside sam goalnode)
    dict11:dict = {(306, 633): 191, (206, 532): 67, (282, 350): 179, (880, 677): 160, (987, 952): 186, (159, 720): 182, (806, 90): 115, (333, 34): 174, (518, 60): 100, (814, 709): 192, (678, 438): 145, (169, 304): 139}
    dict12:dict = {(263, 377): 162, (656, 955): 127, (806, 153): 88, (983, 501): 194, (950, 713): 190, (646, 860): 106, (557, 387): 124, (413, 35): 98, (172, 827): 108, (696, 837): 121, (794, 726): 162, (555, 276): 193}
    dict13:dict = {(810, 622): 65, (211, 669): 65, (941, 415): 196, (464, 712): 145, (673, 740): 187, (219, 27): 160, (726, 384): 165, (173, 343): 183, (799, 583): 111, (429, 207): 133, (258, 771): 166, (730, 505): 102}
    dict14:dict = {(837, 728): 93, (969, 404): 194, (181, 941): 153, (362, 137): 103, (159, 132): 66, (741, 715): 195, (806, 705): 159, (676, 424): 133, (412, 234): 173, (475, 527): 91, (585, 160): 96, (605, 465): 185}
    # crossing riskzones
    dict21:dict = {(171, 489): 122, (881, 106): 63, (509, 507): 176, (719, 372): 150, (793, 472): 147, (453, 919): 103, (244, 91): 51, (375, 843): 156, (411, 712): 76, (948, 197): 183, (876, 490): 120, (818, 184): 155}
    dict22:dict = {(324, 144): 198, (674, 981): 164, (476, 934): 198, (700, 377): 111, (749, 721): 74, (154, 284): 199, (617, 78): 64, (800, 61): 164, (660, 228): 134, (516, 628): 146, (726, 712): 128, (662, 402): 92}
    dict23:dict = {(957, 896): 178, (276, 51): 179, (500, 819): 84, (451, 702): 134, (557, 603): 171, (780, 433): 167, (942, 348): 125, (541, 98): 125, (270, 352): 191, (353, 392): 124, (247, 665): 196, (788, 241): 80}
    # weights
    dict31:dict = {(844, 483): 79, (829, 542): 90, (242, 875): 92, (156, 946): 189, (417, 591): 198, (401, 91): 70, (578, 567): 90, (315, 350): 90, (710, 428): 75, (910, 821): 119, (415, 545): 128, (476, 530): 195}
    # crossing riskzones with level3
    dict41:dict = {(632, 685): 198, (308, 474): 198, (333, 204): 194, (302, 936): 158, (451, 540): 83, (168, 432): 107, (198, 233): 58, (183, 638): 67, (847, 234): 109, (621, 28): 178, (891, 573): 74, (357, 504): 177}
    dict42:dict = {(792, 282): 73, (605, 79): 61, (513, 533): 194, (802, 399): 148, (345, 100): 63, (544, 183): 55, (361, 275): 101, (501, 340): 108, (544, 86): 127, (258, 420): 128, (371, 573): 188, (930, 589): 103}
    # cross sam covering startnode
    dict51:dict = {(292, 855): 154, (402, 766): 177, (982, 774): 74, (165, 41): 189, (206, 811): 152, (233, 53): 126, (780, 891): 166, (972, 306): 176, (503, 601): 84, (269, 846): 141, (775, 143): 66, (576, 205): 57}
    dict52:dict = {(216, 171): 159, (830, 456): 80, (820, 412): 172, (650, 39): 99, (757, 612): 50, (949, 685): 126, (298, 163): 80, (918, 557): 190, (217, 688): 180, (647, 808): 56, (970, 684): 143, (518, 501): 142}
    dict53:dict = {(520, 109): 162, (260, 871): 180, (631, 660): 173, (295, 811): 174, (246, 605): 165, (481, 13): 185, (169, 79): 145, (205, 482): 180, (203, 210): 167, (632, 146): 64, (526, 490): 88, (639, 162): 158}
    # weird goal found ending, first node, not best node
    dict61:dict = {(792, 282): 73, (605, 79): 61, (513, 533): 194, (802, 399): 148, (345, 100): 63, (544, 183): 55, (361, 275): 101, (501, 340): 108, (544, 86): 127, (258, 420): 128, (371, 573): 188, (930, 589): 103}
    
    # init
    riskzones = []
    for item in dict42.items():
        riskzone = RiskZone(item[0],item[1])
        riskzones.append(riskzone)
    # return result
    return riskzones


# generate random location for centre of circle within map dimensions 
def make_random_circle(mapdimensions:tuple[int],
                       riskzone_radius:tuple[int]
                       ) -> tuple[int]:
    # init
    mapheight,mapwidth = mapdimensions
    obstacleradius_min,obstacleradius_max = riskzone_radius
    # draw random values (# todo klopt nog niet helemaal met dimensions en binnen map)
    x_centre = int(random.uniform(150, mapwidth - 10))         
    y_centre = int(random.uniform(10, mapheight - 10))
    radius = int(random.uniform(obstacleradius_min,obstacleradius_max))
    # return result
    return (x_centre,y_centre,radius)