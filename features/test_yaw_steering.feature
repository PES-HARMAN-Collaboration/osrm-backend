Feature: Yaw Rate and Steering Angle Parameters
  As a user of the OSRM match service
  I want to be able to provide yaw_rate and steering_angle parameters
  So that I can improve map matching accuracy with vehicle dynamics data

  Background:
    Given the node map
      | v |   |   |   |   |
      |   |   |   |   |   |
      |   |   |   |   |   |
      |   |   |   |   |   |
      |   |   |   |   |   |
    And the ways
      | nodes | highway |
      | ab    | primary |
      | bc    | primary |
      | cd    | primary |
    And a profile matching car

  Scenario: Match with yaw_rate parameter
    When I match I should get
      | trace | timestamps | yaw_rate | matchings |
      | abc   | 0 10 20   | 5.0;10.0;15.0 | abc |

  Scenario: Match with steering_angle parameter
    When I match I should get
      | trace | timestamps | steering_angle | matchings |
      | abc   | 0 10 20   | 10.0;15.0;20.0 | abc |

  Scenario: Match with both yaw_rate and steering_angle parameters
    When I match I should get
      | trace | timestamps | yaw_rate | steering_angle | matchings |
      | abc   | 0 10 20   | 5.0;10.0;15.0 | 10.0;15.0;20.0 | abc |

  Scenario: Match with optional yaw_rate values
    When I match I should get
      | trace | timestamps | yaw_rate | matchings |
      | abc   | 0 10 20   | 5.0;;15.0 | abc |

  Scenario: Match with optional steering_angle values
    When I match I should get
      | trace | timestamps | steering_angle | matchings |
      | abc   | 0 10 20   | 10.0;;20.0 | abc | 