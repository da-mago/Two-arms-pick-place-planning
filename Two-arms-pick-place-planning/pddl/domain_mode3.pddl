; PDDL PARA MODO 3
; 3 alturas y movimientos diagonales (con down y up)

(define (domain maze)
  (:requirements :strips :typing :equality :negative-preconditions)
  (:types agent agent_pos piece piece_status)
  (:predicates
    (inc_po ?a ?b - piece_status) ; incremento condicion (status) pieza
    (inc ?a ?b - agent_pos) ; incremento en coordenada X o Y o Z
    (dec ?a ?b - agent_pos) ; decremento en coordenada X o Y o Z
    (at ?a - agent ?x ?y ?z - agent_pos) ; amrX en posicion X Y
    (piece_init ?p - piece ?x ?y ?z - agent_pos) ; definicion posicion inicial piezaX
    (piece_end  ?p - piece ?x ?y ?z - agent_pos) ; definicion posicion final piezaX
    (piece_st  ?p - piece ?ps - piece_status) ; Comprobacion estatus pieza
    (carry     ?a - agent) ; armX lleva cogida una pieza - comprobacion
    (carry_map ?a - agent ?p - piece) ; mapeo de que pieza debe recoger un brazo
    (unreachable  ?a - agent ?x ?y ?z - agent_pos) ; posion X Y inalcanzable para armX
    (collision ?a ?b - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 - agent_pos) ; colision entre ambos arm en las posiicon indicadas
    )


;;;  Movimientos ARM1 : back - ARM2 : back  ;;;

(:action move+back+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : back - ARM2 : front  ;;;

(:action move+back+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : back - ARM2 : right  ;;;

(:action move+back+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : back - ARM2 : left  ;;;

(:action move+back+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : back - ARM2 : pick  ;;;

(:action move+back+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p0 ?ps)
                       (inc_po ?ps ?psn)
                       (= p1 ?psn)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                )
  )

;;;  Movimientos ARM1 : back - ARM2 : drop  ;;;

(:action move+back+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p1 ?ps)
                       (inc_po ?ps ?psn)
                       (= p2 ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : back - ARM2 : stay  ;;;

(:action move+back+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : back - ARM2 : down  ;;;

(:action move+back+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : back - ARM2 : up  ;;;

(:action move+back+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : back - ARM2 : left-back  ;;;

(:action move+back+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : back - ARM2 : left-front  ;;;

(:action move+back+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : back - ARM2 : right-back  ;;;

(:action move+back+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : back - ARM2 : right-front  ;;;

(:action move+back+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : back - ARM2 : down_piece  ;;;

(:action move+back+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : back  ;;;

(:action move+front+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : front  ;;;

(:action move+front+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : right  ;;;

(:action move+front+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : left  ;;;

(:action move+front+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : pick  ;;;

(:action move+front+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p0 ?ps)
                       (inc_po ?ps ?psn)
                       (= p1 ?psn)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : drop  ;;;

(:action move+front+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p1 ?ps)
                       (inc_po ?ps ?psn)
                       (= p2 ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : stay  ;;;

(:action move+front+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : down  ;;;

(:action move+front+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : up  ;;;

(:action move+front+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : left-back  ;;;

(:action move+front+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : left-front  ;;;

(:action move+front+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : right-back  ;;;

(:action move+front+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : right-front  ;;;

(:action move+front+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : front - ARM2 : down_piece  ;;;

(:action move+front+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : back  ;;;

(:action move+right+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : front  ;;;

(:action move+right+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : right  ;;;

(:action move+right+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : left  ;;;

(:action move+right+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : pick  ;;;

(:action move+right+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p0 ?ps)
                       (inc_po ?ps ?psn)
                       (= p1 ?psn)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : drop  ;;;

(:action move+right+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p1 ?ps)
                       (inc_po ?ps ?psn)
                       (= p2 ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : stay  ;;;

(:action move+right+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : down  ;;;

(:action move+right+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : up  ;;;

(:action move+right+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : left-back  ;;;

(:action move+right+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : left-front  ;;;

(:action move+right+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : right-back  ;;;

(:action move+right+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : right-front  ;;;

(:action move+right+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right - ARM2 : down_piece  ;;;

(:action move+right+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : back  ;;;

(:action move+left+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : front  ;;;

(:action move+left+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : right  ;;;

(:action move+left+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : left  ;;;

(:action move+left+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : pick  ;;;

(:action move+left+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p0 ?ps)
                       (inc_po ?ps ?psn)
                       (= p1 ?psn)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : drop  ;;;

(:action move+left+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p1 ?ps)
                       (inc_po ?ps ?psn)
                       (= p2 ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : stay  ;;;

(:action move+left+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : down  ;;;

(:action move+left+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : up  ;;;

(:action move+left+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : left-back  ;;;

(:action move+left+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : left-front  ;;;

(:action move+left+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : right-back  ;;;

(:action move+left+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : right-front  ;;;

(:action move+left+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left - ARM2 : down_piece  ;;;

(:action move+left+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : back  ;;;

(:action move+pick+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : front  ;;;

(:action move+pick+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : right  ;;;

(:action move+pick+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : left  ;;;

(:action move+pick+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : pick  ;;;

(:action move+pick+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 - agent_pos ?ps1 ?ps1n ?ps2 ?ps2n - piece_status ?piece1 ?piece2 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece2 ?x2 ?y2 ?z2)
                       (piece_st ?piece2 ?ps2)
                       (= p0 ?ps2)
                       (inc_po ?ps2 ?ps2n)
                       (= p1 ?ps2n)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece2 ?x2 ?y2 ?z2)
                       (not (piece_st ?piece2 ?ps2))
                       (piece_st ?piece2 ?ps2n)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece2)
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : drop  ;;;

(:action move+pick+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 - agent_pos ?ps1 ?ps1n ?ps2 ?ps2n - piece_status ?piece1 ?piece2 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece2 ?x2 ?y2 ?z2)
                       (piece_st ?piece2 ?ps2)
                       (= p1 ?ps2)
                       (inc_po ?ps2 ?ps2n)
                       (= p2 ?ps2n)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece2)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece2 ?x2 ?y2 ?z2)
                       (not (piece_st ?piece2 ?ps2))
                       (piece_st ?piece2 ?ps2n)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : stay  ;;;

(:action move+pick+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : down  ;;;

(:action move+pick+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : up  ;;;

(:action move+pick+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : left-back  ;;;

(:action move+pick+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n ?y2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : left-front  ;;;

(:action move+pick+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n ?y2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : right-back  ;;;

(:action move+pick+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n ?y2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : right-front  ;;;

(:action move+pick+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n ?y2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : pick - ARM2 : down_piece  ;;;

(:action move+pick+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       (not (= ?piece1 ?piece2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p0 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_init ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : back  ;;;

(:action move+drop+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : front  ;;;

(:action move+drop+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : right  ;;;

(:action move+drop+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : left  ;;;

(:action move+drop+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : pick  ;;;

(:action move+drop+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 - agent_pos ?ps1 ?ps1n ?ps2 ?ps2n - piece_status ?piece1 ?piece2 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece2 ?x2 ?y2 ?z2)
                       (piece_st ?piece2 ?ps2)
                       (= p0 ?ps2)
                       (inc_po ?ps2 ?ps2n)
                       (= p1 ?ps2n)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece2 ?x2 ?y2 ?z2)
                       (not (piece_st ?piece2 ?ps2))
                       (piece_st ?piece2 ?ps2n)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece2)
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : drop  ;;;

(:action move+drop+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 - agent_pos ?ps1 ?ps1n ?ps2 ?ps2n - piece_status ?piece1 ?piece2 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece2 ?x2 ?y2 ?z2)
                       (piece_st ?piece2 ?ps2)
                       (= p1 ?ps2)
                       (inc_po ?ps2 ?ps2n)
                       (= p2 ?ps2n)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece2)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece2 ?x2 ?y2 ?z2)
                       (not (piece_st ?piece2 ?ps2))
                       (piece_st ?piece2 ?ps2n)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : stay  ;;;

(:action move+drop+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : down  ;;;

(:action move+drop+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : up  ;;;

(:action move+drop+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : left-back  ;;;

(:action move+drop+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n ?y2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : left-front  ;;;

(:action move+drop+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n ?y2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : right-back  ;;;

(:action move+drop+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n ?y2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : right-front  ;;;

(:action move+drop+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n ?y2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : drop - ARM2 : down_piece  ;;;

(:action move+drop+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z2n - agent_pos ?ps1 ?ps1n - piece_status ?piece1 - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       (not (= ?piece1 ?piece2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (piece_st ?piece1 ?ps1)
                       (= p1 ?ps1)
                       (inc_po ?ps1 ?ps1n)
                       (= p1 ?ps1n)
                       (carry ?omf1)
                       (carry_map ?omf1 ?piece1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (piece_end ?piece1 ?x1 ?y1 ?z1)
                       (not (piece_st ?piece1 ?ps1))
                       (piece_st ?piece1 ?ps1n)
                       (not (carry ?omf1))
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : back  ;;;

(:action move+stay+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : front  ;;;

(:action move+stay+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : right  ;;;

(:action move+stay+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : left  ;;;

(:action move+stay+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : pick  ;;;

(:action move+stay+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p0 ?ps)
                       (inc_po ?ps ?psn)
                       (= p1 ?psn)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : drop  ;;;

(:action move+stay+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p1 ?ps)
                       (inc_po ?ps ?psn)
                       (= p2 ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : stay  ;;;

(:action move+stay+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : down  ;;;

(:action move+stay+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : up  ;;;

(:action move+stay+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : left-back  ;;;

(:action move+stay+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : left-front  ;;;

(:action move+stay+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : right-back  ;;;

(:action move+stay+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : right-front  ;;;

(:action move+stay+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : stay - ARM2 : down_piece  ;;;

(:action move+stay+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : back  ;;;

(:action move+down+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : front  ;;;

(:action move+down+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : right  ;;;

(:action move+down+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : left  ;;;

(:action move+down+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : pick  ;;;

(:action move+down+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p0 ?ps)
                       (inc_po ?ps ?psn)
                       (= p1 ?psn)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : drop  ;;;

(:action move+down+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p1 ?ps)
                       (inc_po ?ps ?psn)
                       (= p2 ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : stay  ;;;

(:action move+down+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : down  ;;;

(:action move+down+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : up  ;;;

(:action move+down+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : left-back  ;;;

(:action move+down+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : left-front  ;;;

(:action move+down+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : right-back  ;;;

(:action move+down+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : right-front  ;;;

(:action move+down+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : down - ARM2 : down_piece  ;;;

(:action move+down+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (not (= z3 ?z1n))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : back  ;;;

(:action move+up+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : front  ;;;

(:action move+up+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : right  ;;;

(:action move+up+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : left  ;;;

(:action move+up+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : pick  ;;;

(:action move+up+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p0 ?ps)
                       (inc_po ?ps ?psn)
                       (= p1 ?psn)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : drop  ;;;

(:action move+up+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p1 ?ps)
                       (inc_po ?ps ?psn)
                       (= p2 ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : stay  ;;;

(:action move+up+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : down  ;;;

(:action move+up+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : up  ;;;

(:action move+up+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : left-back  ;;;

(:action move+up+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : left-front  ;;;

(:action move+up+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : right-back  ;;;

(:action move+up+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : right-front  ;;;

(:action move+up+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : up - ARM2 : down_piece  ;;;

(:action move+up+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?z1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : back  ;;;

(:action move+left-back+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : front  ;;;

(:action move+left-back+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : right  ;;;

(:action move+left-back+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : left  ;;;

(:action move+left-back+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : pick  ;;;

(:action move+left-back+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p0 ?ps)
                       (inc_po ?ps ?psn)
                       (= p1 ?psn)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : drop  ;;;

(:action move+left-back+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p1 ?ps)
                       (inc_po ?ps ?psn)
                       (= p2 ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : stay  ;;;

(:action move+left-back+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : down  ;;;

(:action move+left-back+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : up  ;;;

(:action move+left-back+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : left-back  ;;;

(:action move+left-back+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : left-front  ;;;

(:action move+left-back+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : right-back  ;;;

(:action move+left-back+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : right-front  ;;;

(:action move+left-back+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left-back - ARM2 : down_piece  ;;;

(:action move+left-back+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : back  ;;;

(:action move+left-front+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : front  ;;;

(:action move+left-front+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : right  ;;;

(:action move+left-front+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : left  ;;;

(:action move+left-front+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : pick  ;;;

(:action move+left-front+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p0 ?ps)
                       (inc_po ?ps ?psn)
                       (= p1 ?psn)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : drop  ;;;

(:action move+left-front+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p1 ?ps)
                       (inc_po ?ps ?psn)
                       (= p2 ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : stay  ;;;

(:action move+left-front+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : down  ;;;

(:action move+left-front+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : up  ;;;

(:action move+left-front+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : left-back  ;;;

(:action move+left-front+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : left-front  ;;;

(:action move+left-front+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : right-back  ;;;

(:action move+left-front+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : right-front  ;;;

(:action move+left-front+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : left-front - ARM2 : down_piece  ;;;

(:action move+left-front+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (dec ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : back  ;;;

(:action move+right-back+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : front  ;;;

(:action move+right-back+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : right  ;;;

(:action move+right-back+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : left  ;;;

(:action move+right-back+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : pick  ;;;

(:action move+right-back+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p0 ?ps)
                       (inc_po ?ps ?psn)
                       (= p1 ?psn)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : drop  ;;;

(:action move+right-back+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p1 ?ps)
                       (inc_po ?ps ?psn)
                       (= p2 ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : stay  ;;;

(:action move+right-back+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : down  ;;;

(:action move+right-back+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : up  ;;;

(:action move+right-back+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : left-back  ;;;

(:action move+right-back+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : left-front  ;;;

(:action move+right-back+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : right-back  ;;;

(:action move+right-back+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : right-front  ;;;

(:action move+right-back+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right-back - ARM2 : down_piece  ;;;

(:action move+right-back+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : back  ;;;

(:action move+right-front+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : front  ;;;

(:action move+right-front+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : right  ;;;

(:action move+right-front+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : left  ;;;

(:action move+right-front+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : pick  ;;;

(:action move+right-front+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p0 ?ps)
                       (inc_po ?ps ?psn)
                       (= p1 ?psn)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : drop  ;;;

(:action move+right-front+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p1 ?ps)
                       (inc_po ?ps ?psn)
                       (= p2 ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : stay  ;;;

(:action move+right-front+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : down  ;;;

(:action move+right-front+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : up  ;;;

(:action move+right-front+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : left-back  ;;;

(:action move+right-front+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : left-front  ;;;

(:action move+right-front+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : right-back  ;;;

(:action move+right-front+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : right-front  ;;;

(:action move+right-front+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : right-front - ARM2 : down_piece  ;;;

(:action move+right-front+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?x1n ?y1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (inc ?x1 ?x1n)
                       (inc ?y1 ?y1n)
                       (not (= z3 ?z1))
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1n ?y1n ?z1))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1n ?y1n ?z1 ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1n ?y1n ?z1)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : back  ;;;

(:action move+down_piece+back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : front  ;;;

(:action move+down_piece+front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : right  ;;;

(:action move+down_piece+right
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : left  ;;;

(:action move+down_piece+left
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : pick  ;;;

(:action move+down_piece+pick
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p0 ?ps)
                       (inc_po ?ps ?psn)
                       (= p1 ?psn)
                       (not (carry ?omf2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_init ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : drop  ;;;

(:action move+down_piece+drop
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n - agent_pos ?ps ?psn - piece_status ?piece - piece )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (piece_st ?piece ?ps)
                       (= p1 ?ps)
                       (inc_po ?ps ?psn)
                       (= p2 ?psn)
                       (carry ?omf2)
                       (carry_map ?omf2 ?piece)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (piece_end ?piece ?x2 ?y2 ?z2)
                       (not (piece_st ?piece ?ps))
                       (piece_st ?piece ?psn)
                       (not (carry ?omf2))
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : stay  ;;;

(:action move+down_piece+stay
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : down  ;;;

(:action move+down_piece+down
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (not (= z3 ?z2n))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : up  ;;;

(:action move+down_piece+up
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?z2 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : left-back  ;;;

(:action move+down_piece+left-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : left-front  ;;;

(:action move+down_piece+left-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (dec ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : right-back  ;;;

(:action move+down_piece+right-back
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : right-front  ;;;

(:action move+down_piece+right-front
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?x2n ?y2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (inc ?x2 ?x2n)
                       (inc ?y2 ?y2n)
                       (not (= z3 ?z2))
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2n ?y2n ?z2))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2n ?y2n ?z2))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2n ?y2n ?z2)
                )
  )

;;;  Movimientos ARM1 : down_piece - ARM2 : down_piece  ;;;

(:action move+down_piece+down_piece
    :parameters (?omf1 ?omf2 - agent ?x1 ?y1 ?z1 ?x2 ?y2 ?z2 ?z1n ?z2n - agent_pos )
    :precondition (and
                       (not (= ?omf1 ?omf2))
                       (not (= ?piece1 ?piece2))
                       ;arm1
                       (at ?omf1 ?x1 ?y1 ?z1)
                       (dec ?z1 ?z1n)
                       (= z0 ?z1)
                       (= z3 ?z1n)
                       ;arm2
                       (at ?omf2 ?x2 ?y2 ?z2)
                       (dec ?z2 ?z2n)
                       (= z0 ?z2)
                       (= z3 ?z2n)
                       ;arms collision/reachability
                       (not (unreachable ?omf1 ?x1 ?y1 ?z1n))
                       (not (unreachable ?omf2 ?x2 ?y2 ?z2n))
                       (not (collision ?omf1 ?omf2 ?x1 ?y1 ?z1n ?x2 ?y2 ?z2n))
                  )
    :effect       (and
                       ;arm1
                       (not (at ?omf1 ?x1 ?y1 ?z1))
                       (at ?omf1 ?x1 ?y1 ?z1n)
                       ;arm2
                       (not (at ?omf2 ?x2 ?y2 ?z2))
                       (at ?omf2 ?x2 ?y2 ?z2n)
                )
  )



)