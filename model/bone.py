
bone_parents = [0, 1, 2, 3,    # Thumb: wrist->MCP->PIP->DIP
                0, 5, 6, 7, 8,    # Index
                0, 10, 11, 12, 13,   # Middle
                0, 15, 16, 17, 18  # Ring  
                ]

# bone_parents = [1, 2, 3, 4,    # Thumb: wrist->MCP->PIP->DIP
#                 1, 6, 7, 8, 9,    # Index
#                 1, 11, 12, 13, 14,   # Middle
#                 1, 16, 17, 18, 19  # Ring  
#                 ]    
                    

bone_children = [1, 2, 3, 4,
                 5, 6, 7, 8, 9,
                 10,11,12,13,14,
                 15,16,17,18,19]

# bone_children = [2, 3, 4, 5,
#                  6, 7, 8, 9, 10,
#                  11,12,13,14,15,
#                  16,17,18,19,20]
                 


num_bones = len(bone_parents)  # =20
