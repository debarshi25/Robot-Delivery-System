(define (domain neighborhood)
    
    (:requirements
        :equality
        :typing
        :strips
        :fluents
    )
    
    (:types
        robot
        package
        house
        location
        weight
        binary
    )
    
    (:functions
        (action_cost ?r - robot)
        (x_loc ?l - location)
        (y_loc ?l - location)
    )
    
    (:predicates
        (Package_At ?a - package ?b - location)
        (Package_To ?a - package ?b - location)
        (Package_Size ?a - package ?b - weight)
        (Delivery_Status ?a - package ?b - binary)
        (House_At ?a - house ?b - location)
        (Robot_At ?a - robot ?b - location)
        (Robot_Load ?a - robot ?b - weight)
        (In_Basket ?a - robot ?b - package)
    )
    
    (:action load
        :parameters (?package_id - package
                     ?package_location - location
                     ?package_size - weight
                     ?robot_id - robot
                     ?robot_location - location
                     ?robot_load - weight)
        :precondition
         (and (Package_At ?package_id ?package_location) (Package_Size ?package_id ?package_size) (Delivery_Status ?package_id False) (Robot_At ?robot_id ?robot_location) (Robot_Load ?robot_id ?robot_load) (not(In_Basket ?robot_id ?package_id)) (= ?package_location ?robot_location))
        :effect
         (and
                (when (and (= ?package_size n25) (= ?robot_load n0))
                    (and
                        (In_Basket ?robot_id ?package_id)
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n25)
                        (not (Package_At ?package_id ?package_location))
                    )
                )
                (when (and (= ?package_size n25) (= ?robot_load n25))
                    (and
                        (In_Basket ?robot_id ?package_id)
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n50)
                        (not (Package_At ?package_id ?package_location))
                    )
                )
                (when (and (= ?package_size n25) (= ?robot_load n50))
                    (and
                        (In_Basket ?robot_id ?package_id)
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n75)
                        (not (Package_At ?package_id ?package_location))
                    )
                )
                (when (and (= ?package_size n25) (= ?robot_load n75))
                    (and
                        (In_Basket ?robot_id ?package_id)
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n100)
                        (not (Package_At ?package_id ?package_location))
                    )
                )
                (when (and (= ?package_size n50) (= ?robot_load n0))
                    (and
                        (In_Basket ?robot_id ?package_id)
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n50)
                        (not (Package_At ?package_id ?package_location))
                    )
                )
                (when (and (= ?package_size n50) (= ?robot_load n25))
                    (and
                        (In_Basket ?robot_id ?package_id)
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n75)
                        (not (Package_At ?package_id ?package_location))
                    )
                )
                (when (and (= ?package_size n50) (= ?robot_load n50))
                    (and
                        (In_Basket ?robot_id ?package_id)
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n100)
                        (not (Package_At ?package_id ?package_location))
                    )
                )
                (when (and (= ?package_size n75) (= ?robot_load n0))
                    (and
                        (In_Basket ?robot_id ?package_id)
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n75)
                        (not (Package_At ?package_id ?package_location))
                    )
                )
                (when (and (= ?package_size n75) (= ?robot_load n25))
                    (and
                        (In_Basket ?robot_id ?package_id)
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n100)
                        (not (Package_At ?package_id ?package_location))
                    )
                )
                (when (and (= ?package_size n100) (= ?robot_load n0))
                    (and
                        (In_Basket ?robot_id ?package_id)
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n100)
                        (not (Package_At ?package_id ?package_location))
                    )
                ))
    )
    
    (:action deliver
        :parameters (
            ?package_id - package
            ?package_size - weight
            ?house_id - house
            ?house_location - location
            ?robot_id - robot
            ?robot_location - location
            ?robot_load - weight
        )
        :precondition
            (and
                (Package_Size ?package_id ?package_size)
                (Delivery_Status ?package_id False)
                (Package_To ?package_id ?house_location)
                (House_At ?house_id ?house_location)
                (Robot_At ?robot_id ?robot_location)
                (Robot_Load ?robot_id ?robot_load)
                (= ?house_location ?robot_location)
                (In_Basket ?robot_id ?package_id)
            )
        
        :effect
            (and
                (when (and (= ?package_size n25) (= ?robot_load n25))
                    (and
                        (not (In_Basket ?robot_id ?package_id))
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n0)
                        (Package_At ?package_id ?house_location)
                        (Delivery_Status ?package_id True)
                        (not (Delivery_Status ?package_id False))
                    )
                )
                (when (and (= ?package_size n25) (= ?robot_load n50))
                    (and
                        (not (In_Basket ?robot_id ?package_id))
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n25)
                        (Package_At ?package_id ?house_location)
                        (Delivery_Status ?package_id True)
                        (not (Delivery_Status ?package_id False))
                    )
                )
                (when (and (= ?package_size n25) (= ?robot_load n75))
                    (and
                        (not (In_Basket ?robot_id ?package_id))
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n50)
                        (Package_At ?package_id ?house_location)
                        (Delivery_Status ?package_id True)
                        (not (Delivery_Status ?package_id False))
                    )
                )
                (when (and (= ?package_size n25) (= ?robot_load n100))
                    (and
                        (not (In_Basket ?robot_id ?package_id))
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n75)
                        (Package_At ?package_id ?house_location)
                        (Delivery_Status ?package_id True)
                        (not (Delivery_Status ?package_id False))
                    )
                )
                (when (and (= ?package_size n50) (= ?robot_load n50))
                    (and
                        (not (In_Basket ?robot_id ?package_id))
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n0)
                        (Package_At ?package_id ?house_location)
                        (Delivery_Status ?package_id True)
                        (not (Delivery_Status ?package_id False))
                    )
                )
                (when (and (= ?package_size n50) (= ?robot_load n75))
                    (and
                        (not (In_Basket ?robot_id ?package_id))
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n25)
                        (Package_At ?package_id ?house_location)
                        (Delivery_Status ?package_id True)
                        (not (Delivery_Status ?package_id False))
                    )
                )
                (when (and (= ?package_size n50) (= ?robot_load n100))
                    (and
                        (not (In_Basket ?robot_id ?package_id))
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n50)
                        (Package_At ?package_id ?house_location)
                        (Delivery_Status ?package_id True)
                        (not (Delivery_Status ?package_id False))
                    )
                )
                (when (and (= ?package_size n75) (= ?robot_load n75))
                    (and
                        (not (In_Basket ?robot_id ?package_id))
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n0)
                        (Package_At ?package_id ?house_location)
                        (Delivery_Status ?package_id True)
                        (not (Delivery_Status ?package_id False))
                    )
                )
                (when (and (= ?package_size n75) (= ?robot_load n100))
                    (and
                        (not (In_Basket ?robot_id ?package_id))
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n25)
                        (Package_At ?package_id ?house_location)
                        (Delivery_Status ?package_id True)
                        (not (Delivery_Status ?package_id False))
                    )
                )
                (when (and (= ?package_size n100) (= ?robot_load n100))
                    (and
                        (not (In_Basket ?robot_id ?package_id))
                        (not (Robot_Load ?robot_id ?robot_load))
                        (Robot_Load ?robot_id n0)
                        (Package_At ?package_id ?house_location)
                        (Delivery_Status ?package_id True)
                        (not (Delivery_Status ?package_id False))
                    )
                )
            )
    )
    
    (:action move
        :parameters (
            ?loc_from - location
            ?loc_to - location
            ?robot_id - robot
        )
        :precondition
            (and
                (Robot_At ?robot_id ?loc_from)
                (not (= ?loc_from ?loc_to))
                (not (exists (?r - robot) (Robot_At ?r ?loc_to)))
                (< (action_cost ?robot_id) 100)
            )
        
        :effect
            (and
                (Robot_At ?robot_id ?loc_to)
                (not (Robot_At ?robot_id ?loc_from))
                (when   (and
                            (<= (x_loc ?loc_from) (x_loc ?loc_to))
                            (<= (y_loc ?loc_from) (y_loc ?loc_to))
                        )

                        (increase (action_cost ?robot_id) (+ (- (x_loc ?loc_to) (x_loc ?loc_from)) (- (y_loc ?loc_to) (y_loc ?loc_from))))
                )
                (when   (and
                            (> (x_loc ?loc_from) (x_loc ?loc_to))
                            (<= (y_loc ?loc_from) (y_loc ?loc_to))
                        )

                        (increase (action_cost ?robot_id) (+ (- (x_loc ?loc_from) (x_loc ?loc_to)) (- (y_loc ?loc_to) (y_loc ?loc_from))))
                )
                (when   (and
                            (<= (x_loc ?loc_from) (x_loc ?loc_to))
                            (> (y_loc ?loc_from) (y_loc ?loc_to))
                        )

                        (increase (action_cost ?robot_id) (+ (- (x_loc ?loc_to) (x_loc ?loc_from)) (- (y_loc ?loc_from) (y_loc ?loc_to))))
                )
                (when   (and
                            (> (x_loc ?loc_from) (x_loc ?loc_to))
                            (> (y_loc ?loc_from) (y_loc ?loc_to))
                        )

                        (increase (action_cost ?robot_id) (+ (- (x_loc ?loc_from) (x_loc ?loc_to)) (- (y_loc ?loc_from) (y_loc ?loc_to))))
                )
            )
    )
)
