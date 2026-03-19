"""
File: dubins_car.py

Author: Damien ESNAULT (PhD student)
Institution: ENSTA/Lab-STICC
Date: 2025

Summary:
    Minimal and reusable 2D Dubins-car simulator for scientific computing,
    Monte Carlo sample generation, and robotics prototyping.

    This module provides a lightweight base class for simulating a planar
    Dubins-car-like system with RK2 integration and trajectory recording.
    It is intentionally generic and unconstrained so that users can easily
    adapt it to their own applications by overriding the command law and
    mission completion criterion.

Key Features:
    - 2D Dubins-car state: [px, py, heading, speed]
    - Control input: [heading_rate (rad/s), acceleration (m/s²)]
    - Second-order Runge-Kutta (RK2) integration
    - Time-stepped simulation with user-defined mission stopping condition
    - Automatic recording of state and time trajectories
    - Reset mechanism for repeated single-shot simulations
    - Designed to be subclassed or customized dynamically with MethodType
    - Suitable for Monte Carlo generation of trajectory samples

Design Philosophy:
    - Keep the base model simple, transparent, and easy to reuse
    - Do not impose application-specific assumptions in the core class
    - Let users define:
        * the command law,
        * the mission completion condition,
        * any additional constraints or stochastic effects

Important Notes:
    - The class does not normalize the heading angle.
    - The class does not prevent negative speed values.
    - If needed, such constraints must be handled by the user in the
      overridden compute_command_vector() method or in a derived class.
    - If the simulation time step does not exactly divide the maximal
      duration, the simulation is propagated until the first sampled time
      greater than or equal to the requested final time.

Intended Users:
    - Robotics students, researchers, and engineers
    - Users needing a simple base model for trajectory simulation
    - Users generating Monte Carlo samples for downstream processing
    - Beginner users who want to override methods without writing a full subclass
"""

import numpy as np
import math
from typing import Tuple, List, Union

class DubinsCar:
    """
    Minimal and reusable 2D Dubins-car dynamical system.

    This class provides a lightweight base model for simulating planar vehicle
    trajectories of Dubins-car type. It is designed for general trajectory
    simulation in scientific and robotics applications, while remaining simple
    enough to be reused and adapted in custom projects.

    The system state is represented by:
        state_vector = [px, py, heading, speed]
            - px, py   : planar position coordinates (meters)
            - heading  : vehicle orientation (radians)
            - speed    : linear speed (m/s)

    The control input is represented by:
        command_vector = [heading_rate, acceleration]
            - heading_rate : angular velocity (rad/s)
            - acceleration : longitudinal acceleration (m/s²)

    The vehicle dynamics are propagated using a second-order Runge-Kutta (RK2)
    integration scheme, and the full simulated trajectory is automatically
    recorded during simulation.

    Recorded trajectories are available through:
        - self.state_vector_trajectory
            List of recorded state vectors, one per simulated time step
        - self.time_trajectory
            List of recorded simulation times associated with each state

    These recorded trajectories can be accessed directly after calling
    run_simulation(), for example for plotting, exporting, or statistical analysis.

    Important design notes:
        - The class does not normalize the heading angle.
        - The class does not prevent negative speed values.
        - Such constraints, if required, must be handled by the user in the
          overridden compute_command_vector() method or in a derived class.

    User responsibilities:
        This class is intentionally incomplete as a simulation framework.
        To use it, the user must override the two following virtual methods:
            - compute_command_vector()
            - mission_is_completed()

        Without these two methods, the simulator cannot be used.

    Usage philosophy:
        - The intended use is one instance = one trajectory.
        - The class also supports multiple successive simulations through reset(),
          but previously recorded trajectories are then cleared.
        - If multiple trajectories must be preserved, they should be saved by the
          user outside the instance before running a new simulation.

    This design makes the class suitable as:
        - a simple educational Dubins-car simulator,
        - a reusable base class for more advanced vehicle models,
        - a building block for higher-level simulation pipelines.
    """
    
    def __init__(self, initial_state_vector: Union[Tuple[float,float,float,float],List[float],np.ndarray]=(0.0, 0.0, 0.0, 0.0)):
        """
        Initialize a DubinsCar instance.

        This constructor defines the reference initial state of the vehicle and
        initializes the current state accordingly. The provided initial state is
        stored in two forms:

            - self.initial_state_vector:
                reference initial condition used when calling reset()
            - self.state_vector:
                current mutable state of the vehicle during simulation

        The initial state must contain exactly four components ordered as:
            [px0, py0, heading0, speed0]

        Args:
            initial_state_vector (tuple, list, or np.ndarray, optional):
                Initial vehicle state [px0, py0, heading0, speed0], where:
                    - px0      : initial x position (m)
                    - py0      : initial y position (m)
                    - heading0 : initial heading (rad)
                    - speed0   : initial speed (m/s)

                Defaults to (0.0, 0.0, 0.0, 0.0).

        Raises:
            TypeError:
                If initial_state_vector cannot be converted to a flat numpy array
                of floats.

            ValueError:
                If initial_state_vector does not contain exactly 4 elements.

        Notes:
            - The constructor does not normalize the heading angle.
            - The constructor does not impose any constraint on the speed value.
            - Recorded trajectories are initialized as empty lists and will be
            populated only when run_simulation() is called.
        """
        # Convert the user-provided initial state into a flat numpy array of floats.
        # This ensures a consistent internal representation whatever the input type
        # (tuple, list, or numpy array).
        try:
            initial_state_vector = np.array(initial_state_vector, dtype=float).flatten()
        except Exception as exc:
            raise TypeError("DubinsCar.__init__(): unable to convert initial_state_vector to a flat numpy array of floats.") from exc
        
        # Validate the state dimension.
        # The Dubins-car model expects exactly four state variables:
        # [px, py, heading, speed].
        if initial_state_vector.size != 4:
            raise ValueError("DubinsCar.__init__(): initial_state_vector must contain exactly 4 elements ordered as [px, py, heading, speed].")

        # Store the reference initial state.
        # This vector is kept unchanged unless reset(initial_state_vector=...)
        # is explicitly called by the user.
        self.initial_state_vector = initial_state_vector.copy()

        # Initialize the current mutable state from the reference initial state.
        # This state will evolve during simulation.
        self.state_vector = self.initial_state_vector.copy()

        # Initialize recorded trajectories as empty containers.
        # They will be filled by run_simulation().
        self.state_vector_trajectory = []
        self.time_trajectory = []
    
    def evolution_function(self, state_vector: np.ndarray, command_vector: np.ndarray) -> np.ndarray:
        """
        Compute the time derivative of the Dubins-car state.

        This method evaluates the continuous-time dynamics of the system for a given
        state and control input.

        The state vector is defined as:
            state_vector = [px, py, heading, speed]

        The command vector is defined as:
            command_vector = [heading_rate, acceleration]

        The associated dynamics are:
            px_dot      = speed * cos(heading)
            py_dot      = speed * sin(heading)
            heading_dot = heading_rate
            speed_dot   = acceleration

        Args:
            state_vector (np.ndarray):
                Current state vector [px, py, heading, speed].

            command_vector (np.ndarray):
                Current command vector [heading_rate, acceleration].

        Returns:
            np.ndarray:
                State derivative vector [px_dot, py_dot, heading_dot, speed_dot].

        Raises:
            TypeError:
                If state_vector or command_vector cannot be converted to flat numpy
                arrays of floats.

            ValueError:
                If state_vector does not contain exactly 4 elements or if
                command_vector does not contain exactly 2 elements.

        Notes:
            - This method evaluates the unconstrained Dubins-car dynamics.
            - No heading normalization is applied.
            - No constraint is imposed on the speed value.
        """
        # Convert the input state into a flat numpy array of floats so that the
        # internal computation remains robust and independent of the input type.
        try:
            state_vector = np.array(state_vector, dtype=float).flatten()
        except Exception as exc:
            raise TypeError("DubinsCar.evolution_function(): unable to convert state_vector to a flat numpy array of floats.") from exc
        
        # Validate the state dimension.
        # The expected order is: [px, py, heading, speed].
        if state_vector.size != 4:
            raise ValueError("DubinsCar.evolution_function(): state_vector must contain exactly 4 elements ordered as [px, py, heading, speed].")
        
        # Convert the input command into a flat numpy array of floats so that the
        # method accepts tuples, lists, and numpy arrays transparently.
        try:
            command_vector = np.array(command_vector, dtype=float).flatten()
        except Exception as exc:
            raise TypeError("DubinsCar.evolution_function(): unable to convert command_vector to a flat numpy array of floats.") from exc
        
        # Validate the command dimension.
        # The expected order is: [heading_rate, acceleration].
        if command_vector.size != 2:
            raise ValueError("DubinsCar.evolution_function(): command_vector must contain exactly 2 elements ordered as [heading_rate, acceleration].")

        # Unpack state and command variables.
        px, py, heading, speed = state_vector
        command_heading, command_speed = command_vector

         # Evaluate the Dubins-car state derivatives.
        dpx = speed*np.cos(heading)
        dpy = speed*np.sin(heading)
        dheading = command_heading
        dspeed = command_speed

        # Return the state derivative as a flat numpy array.
        return np.array([dpx, dpy, dheading, dspeed], dtype=float).flatten()
    
    def RK2_integration_scheme(self, state_vector: np.ndarray, command_vector: np.ndarray, integration_time_step: float) -> np.ndarray:
        """
        Integrate the Dubins-car dynamics over one time step using a second-order Runge-Kutta (RK2) scheme.

        This method performs a single numerical integration step of the continuous-time
        Dubins-car model using the midpoint Runge-Kutta method (RK2). The RK2 method
        provides improved accuracy compared to the Euler method while remaining
        computationally lightweight.

        The integration is performed according to the following procedure:

            k1 = f(x_k, u_k)
            k2 = f(x_k + dt * k1, u_k)

            x_{k+1} = x_k + 0.5 * dt * (k1 + k2)

        where:
            - x_k is the current state
            - u_k is the command vector
            - dt is the integration time step
            - f(.) is the system evolution function

        Args:
            state_vector (np.ndarray):
                Current system state [px, py, heading, speed].

            command_vector (np.ndarray):
                Current command vector [heading_rate, acceleration].

            integration_time_step (float):
                Integration time step (seconds).

        Returns:
            np.ndarray:
                Updated state vector after one integration step.

        Raises:
            TypeError:
                If state_vector or command_vector cannot be converted to flat numpy arrays
                or if integration_time_step cannot be converted to a float.

            ValueError:
                If state_vector does not contain exactly 4 elements, if command_vector
                does not contain exactly 2 elements, or if integration_time_step is not positive.
        """
        # Convert and validate the state vector
        try:
            state_vector = np.array(state_vector, dtype=float).flatten()
        except Exception as exc:
            raise TypeError("DubinsCar.RK2_integration_scheme(): unable to convert state_vector to a flat numpy array of floats.") from exc

        if state_vector.size != 4:
            raise ValueError("DubinsCar.RK2_integration_scheme(): state_vector must contain exactly 4 elements ordered as [px, py, heading, speed].")

        # Convert and validate the command vector
        try:
            command_vector = np.array(command_vector, dtype=float).flatten()
        except Exception as exc:
            raise TypeError("DubinsCar.RK2_integration_scheme(): unable to convert command_vector to a flat numpy array of floats.") from exc

        if command_vector.size != 2:
            raise ValueError("DubinsCar.RK2_integration_scheme(): command_vector must contain exactly 2 elements ordered as [heading_rate, acceleration].")

        # Convert and validate the integration time step
        try:
            integration_time_step = float(integration_time_step)
        except Exception as exc:
            raise TypeError("DubinsCar.RK2_integration_scheme(): unable to convert integration_time_step to float.") from exc

        if integration_time_step <= 0:
            raise ValueError("DubinsCar.RK2_integration_scheme(): integration_time_step must be strictly greater than 0.")
        
        # First RK2 slope evaluation
        k1 = self.evolution_function(state_vector, command_vector)

        # Second RK2 slope evaluation (midpoint prediction)
        k2 = self.evolution_function(state_vector + integration_time_step*k1, command_vector)

        # RK2 state update
        new_state_vector = state_vector + 0.5*integration_time_step*(k1+k2)

        return new_state_vector
    
    def compute_command_vector(self, state_vector: np.ndarray, simulation_time: float) -> np.ndarray:
        """
        Virtual method used to compute the control input applied to the vehicle.

        This method defines the command law of the Dubins-car system. It is intentionally
        left unimplemented in the base class and must be overridden by the user before
        calling run_simulation().

        At each simulation step, this method is called with:
            - the current vehicle state,
            - the current simulation time,

        and must return the command vector to apply during the next integration step.

        Expected input:
            state_vector = [px, py, heading, speed]
                - px      : current x position (m)
                - py      : current y position (m)
                - heading : current heading angle (rad)
                - speed   : current linear speed (m/s)

            simulation_time:
                Current simulation time (s)

        Expected output:
            command_vector = [heading_rate, acceleration]
                - heading_rate : angular velocity command (rad/s)
                - acceleration : longitudinal acceleration command (m/s²)

        Important notes for users:
            - This method must be overridden before run_simulation() is called.
            - The base class does not impose any constraint on heading or speed.
            - In particular, if the user wants to prevent negative speed values,
            this must be handled in the command law implemented here.
            - The returned command must be compatible with the expected Dubins-car model.

        Typical usage patterns:
            1) Override the method dynamically for a single instance using MethodType.
            2) Override the method in a derived class (recommended for reusable code).

        Example 1: override for a single instance
            from types import MethodType

            def new_compute_command_vector_method(self, state_vector, simulation_time):
                px, py, heading, speed = np.array(state_vector, dtype=float).flatten()
                heading_rate = 0.0
                acceleration = 0.0
                return np.array([heading_rate, acceleration], dtype=float)

            dubins_car = DubinsCar()
            dubins_car.compute_command_vector = MethodType(new_compute_command_vector_method, dubins_car)

        Example 2: override in a derived class
            class MyCustomDubinsCar(DubinsCar):
                def compute_command_vector(self, state_vector, simulation_time):
                    px, py, heading, speed = np.array(state_vector, dtype=float).flatten()
                    heading_rate = 0.0
                    acceleration = 0.0
                    return np.array([heading_rate, acceleration], dtype=float)

            custom_dubins_car = MyCustomDubinsCar()

        Args:
            state_vector (np.ndarray):
                Current system state [px, py, heading, speed].

            simulation_time (float):
                Current simulation time (s).

        Returns:
            np.ndarray:
                Command vector [heading_rate, acceleration].

        Raises:
            NotImplementedError:
                Always, until the user overrides this method.
        """
        raise NotImplementedError("DubinsCar.compute_command_vector(): this virtual method must be overridden before calling run_simulation().")

    def mission_is_completed(self, state_vector: np.ndarray, simulation_time: float) -> bool:
        """
        Virtual method used to determine whether the simulation mission is completed.

        This method defines the stopping condition of the simulation. It is intentionally
        left unimplemented in the base class and must be overridden by the user before
        calling run_simulation().

        At each simulation step, this method is called with:
            - the current vehicle state,
            - the current simulation time,

        and must return a boolean indicating whether the mission is considered complete.

        Expected input:
            state_vector = [px, py, heading, speed]
                - px      : current x position (m)
                - py      : current y position (m)
                - heading : current heading angle (rad)
                - speed   : current linear speed (m/s)

            simulation_time:
                Current simulation time (s)

        Expected output:
            mission_state:
                - True  : the mission is completed and the simulation must stop
                - False : the mission is still in progress and the simulation continues

        Important notes for users:
            - This method must be overridden before run_simulation() is called.
            - The stopping criterion is entirely user-defined.
            - Typical stopping conditions may depend on:
                * the simulated time,
                * the vehicle position,
                * the distance to a target,
                * the completion of a guidance objective,
                * or any other mission-specific condition.

        Typical usage patterns:
            1) Override the method dynamically for a single instance using MethodType.
            2) Override the method in a derived class (recommended for reusable code).

        Example 1: override for a single instance
            from types import MethodType

            def new_mission_is_completed_method(self, state_vector, simulation_time):
                return simulation_time >= 10.0

            dubins_car = DubinsCar()
            dubins_car.mission_is_completed = MethodType(new_mission_is_completed_method, dubins_car)

        Example 2: override in a derived class
            class MyCustomDubinsCar(DubinsCar):
                def mission_is_completed(self, state_vector, simulation_time):
                    px, py, heading, speed = np.array(state_vector, dtype=float).flatten()
                    return np.linalg.norm([px, py]) >= 10.0

            custom_dubins_car = MyCustomDubinsCar()

        Args:
            state_vector (np.ndarray):
                Current system state [px, py, heading, speed].

            simulation_time (float):
                Current simulation time (s).

        Returns:
            bool:
                True if the mission is completed, False otherwise.

        Raises:
            NotImplementedError:
                Always, until the user overrides this method.
        """
        raise NotImplementedError("DubinsCar.mission_is_completed(): this virtual method must be overridden before calling run_simulation().")

    def reset(self, initial_state_vector: Union[Tuple[float,float,float,float], List[float], np.ndarray, None] = None):
        """
        Reset the simulator state and recorded trajectories.

        This method clears the recorded simulation data and restores the internal
        vehicle state to the reference initial condition stored in
        `self.initial_state_vector`.

        Optionally, the user may provide a new initial state vector. In that case,
        the reference initial state of the simulator is updated before resetting
        the current state.

        After calling reset():
            - self.state_vector is set to the reference initial state
            - self.state_vector_trajectory is cleared
            - self.time_trajectory is cleared

        This method is typically used to prepare the simulator for a new trajectory
        simulation.

        Args:
            initial_state_vector (tuple, list, np.ndarray, optional):
                New initial state [px0, py0, heading0, speed0]. If provided, this
                replaces the previously stored reference initial state.

        Raises:
            TypeError:
                If the provided initial_state_vector cannot be converted to a flat
                numpy array of floats.

            ValueError:
                If the provided initial_state_vector does not contain exactly
                4 elements.
        """ 
        # Clear previously recorded trajectories
        self.time_trajectory = []
        self.state_vector_trajectory = []

        # If the user provides a new initial state, update the reference state
        if initial_state_vector is not None:

            # Convert the provided initial state into a flat numpy array
            try:
                initial_state_vector = np.array(initial_state_vector, dtype=float).flatten()
            except Exception as exc:
                raise TypeError("DubinsCar.reset(): unable to convert initial_state_vector to a flat numpy array of floats.") from exc

            # Validate the state dimension
            if initial_state_vector.size != 4:
                raise ValueError("DubinsCar.reset(): initial_state_vector must contain exactly 4 elements ordered as [px, py, heading, speed].")

            # Update the stored reference initial state
            self.initial_state_vector = initial_state_vector

        # Restore the current state from the stored reference initial state
        self.state_vector = self.initial_state_vector.copy()

    def run_simulation(self, simulation_maximal_duration: float, simulation_time_step: float, simulation_initial_time: float = 0., debug_print: bool = False):
        """
        Run a full trajectory simulation of the Dubins-car system.

        This method performs a time-stepped simulation starting from the reference
        initial state stored in the instance. Before starting, the simulator is
        reset, meaning that:
            - the current state is restored to the reference initial state,
            - previously recorded trajectories are cleared.

        The simulation then proceeds iteratively as follows:
            1) initialize the trajectory with the initial state and initial time,
            2) evaluate whether the mission is already completed,
            3) if not, compute the command vector at the current time,
            4) propagate the state over one time step using the RK2 integration scheme,
            5) record the updated state and time,
            6) evaluate the mission completion criterion again,
            7) repeat until the mission is completed or the maximal duration is reached.

        The simulation stops when either:
            - mission_is_completed() returns True,
            - or the maximal simulation duration has been covered.

        Important note:
            If simulation_time_step is not an exact divisor of simulation_maximal_duration,
            the simulation is propagated until the first sampled time greater than or
            equal to simulation_initial_time + simulation_maximal_duration.

        Recorded outputs:
            After the simulation, the full trajectory can be accessed through:
                - self.state_vector_trajectory
                - self.time_trajectory

        Args:
            simulation_maximal_duration (float):
                Maximum simulation duration in seconds. Must be strictly positive.

            simulation_time_step (float):
                Integration time step in seconds. Must be strictly positive.

            simulation_initial_time (float, optional):
                Initial simulation time in seconds. Must be greater than or equal to 0.
                Defaults to 0.0.

            debug_print (bool, optional):
                If True, display simulation progress and a final overview in the terminal.
                Defaults to False.

        Raises:
            TypeError:
                If simulation_maximal_duration, simulation_time_step, or
                simulation_initial_time cannot be converted to float, or if
                debug_print is not a boolean, or if the return values of
                compute_command_vector() or mission_is_completed() have invalid types.

            ValueError:
                If simulation_maximal_duration or simulation_time_step are not strictly
                positive, if simulation_initial_time is negative, or if the command
                vector returned by compute_command_vector() does not contain exactly
                2 elements.

        Notes:
            - This method assumes that compute_command_vector() and mission_is_completed()
            have been overridden by the user before calling run_simulation().
            - The simulator is reset automatically at the beginning of the method.
            - The generated trajectory replaces any previously recorded one.
        """
        # Verify the type of simulation_maximal_duration
        # Convert and validate the maximal simulation duration
        try:
            mission_duration = float(simulation_maximal_duration)
        except Exception as exc:
            raise TypeError("DubinsCar.run_simulation(): unable to convert simulation_maximal_duration to float.") from exc
        if mission_duration <= 0:
            raise ValueError("DubinsCar.run_simulation(): simulation_maximal_duration must be strictly greater than 0.")

        # Convert and validate the simulation time step
        try:
            time_step = float(simulation_time_step)
        except Exception as exc:
            raise TypeError("DubinsCar.run_simulation(): unable to convert simulation_time_step to float.") from exc
        if time_step <= 0:
            raise ValueError("DubinsCar.run_simulation(): simulation_time_step must be strictly greater than 0.")

        # Convert and validate the initial simulation time
        try:
            initial_time = float(simulation_initial_time)
        except Exception as exc:
            raise TypeError("DubinsCar.run_simulation(): unable to convert simulation_initial_time to float.") from exc
        if initial_time < 0:
            raise ValueError("DubinsCar.run_simulation(): simulation_initial_time must be greater than or equal to 0.")

        # Validate the debug flag
        if not isinstance(debug_print, bool):
            raise TypeError("DubinsCar.run_simulation(): debug_print must be a boolean.")

        # Reset the simulator so the run starts from the stored reference initial state
        # and from empty trajectory buffers.
        self.reset()

        # Initialize the recorded trajectory with the initial state and initial time
        self.time_trajectory.append(initial_time)
        self.state_vector_trajectory.append(tuple(self.state_vector))

        # Build the vector of sampled simulation times.
        # If the time step does not exactly divide the mission duration, the final
        # sampled time may be greater than the requested final time.
        n_time_step = math.ceil(mission_duration / time_step)
        simulation_time_vector = initial_time + time_step * np.arange(n_time_step + 1)

         # Evaluate the mission state at the initial condition
        mission_completion = self.mission_is_completed(self.state_vector, initial_time)
        # Check that the mission completion function returns a boolean
        if not isinstance(mission_completion, bool):
            raise TypeError("DubinsCar.run_simulation(): mission_is_completed() must return a boolean.")
        
        # Optional live progress display
        if debug_print:
            print("[DEBUG] Simulation time: {:.2f}s/{:.2f}s".format(initial_time, initial_time + mission_duration), end="")

        # Main simulation loop
        for k in range(len(simulation_time_vector)-1):
            
            # Stop immediately if the mission has already been completed
            if mission_completion:
                break
            
            # Compute the command vector at the current simulation time
            command_vector = self.compute_command_vector(self.state_vector, simulation_time_vector[k])

            # Convert the returned command into a flat numpy array
            try:
                command_vector = np.array(command_vector, dtype=float).flatten()
            except Exception as exc:
                raise TypeError("DubinsCar.run_simulation(): unable to convert the command returned by compute_command_vector() to a flat numpy array of floats.") from exc
            # Validate the command dimension
            if command_vector.size != 2:
                raise ValueError("DubinsCar.run_simulation(): compute_command_vector() must return exactly 2 elements ordered as [heading_rate, acceleration].")

            # Propagate the system state over one time step
            self.state_vector = self.RK2_integration_scheme(self.state_vector, command_vector, time_step)

            # Record the updated state and associated simulation time
            self.time_trajectory.append(simulation_time_vector[k+1])
            self.state_vector_trajectory.append(tuple(self.state_vector))

            # Evaluate the mission completion criterion after the state update
            mission_completion = self.mission_is_completed(self.state_vector, simulation_time_vector[k+1])
            
            # Check that the mission completion function returns a boolean
            if not isinstance(mission_completion, bool):
                raise TypeError("DubinsCar.run_simulation(): mission_is_completed() must return a boolean.")

            # Optional live progress display
            if debug_print:
                print("\r[DEBUG] Simulation time: {:.2f}s/{:.2f}s".format(simulation_time_vector[k+1], initial_time+mission_duration), end="")
        
        # Optional final summary
        if debug_print:
            print("\n[DEBUG] Simulation overview:")
            print("\tStart time: {:.2f}s".format(self.time_trajectory[0]))
            print("\tEnd time: {:.2f}s".format(self.time_trajectory[-1]))
            print("\tTime step: {:.2f}s".format(time_step))
            print("\tNumber of time points: {}".format(len(self.time_trajectory)))
            print("\tNumber of state points: {}".format(len(self.state_vector_trajectory)))
            print("\tInitial state: {}".format(self.state_vector_trajectory[0]))
            print("\tFinal state: {}".format(self.state_vector_trajectory[-1]))



# ====================================================
# EXAMPLE: Tracking a moving target (Lissajous curve)
# ====================================================
#
# This example demonstrates how to use the DubinsCar class to simulate
# a vehicle tracking a time-varying target.
#
# The target follows a two-dimensional Lissajous curve shaped like an "eight",
# centered at position (5, 3). The vehicle uses:
#   - a proportional heading controller to orient itself toward the target,
#   - a proportional speed controller to adapt its velocity based on the
#     distance to the target.
#
# The objective of this example is to illustrate:
#   - how to override compute_command_vector() and mission_is_completed(),
#   - how to implement a simple trajectory tracking controller,
#   - how to simulate a Dubins-car following a dynamic reference,
#   - how to visualize both the vehicle trajectory and the target trajectory.
#
# Expected result:
#   - the target trajectory appears as an "eight" curve,
#   - the Dubins car follows this curve with a tracking delay,
#     depending on controller gains and target speed.
#
# This example provides a more dynamic and visually informative behavior
# than a simple straight-line or constant-heading motion.
#
# ====================================================

if __name__ == "__main__":
    from types import MethodType
    import matplotlib.pyplot as plt

    def target_position(simulation_time):
        """
        Return the position of the moving target describing an
        eight-shaped Lissajous curve centered on (5, 3).
        """
        cx = 5.0
        cy = 3.0
        ax = 2.0
        ay = 1.0
        omega = 0.2

        return np.array([
            cx + ax * np.sin(omega * (simulation_time - 5.0)),
            cy + ay * np.sin(2.0 * omega * (simulation_time - 5.0))
        ], dtype=float)

    def new_compute_command_vector_method(self, state_vector, simulation_time):
        """
        Compute the command vector for tracking a moving target describing
        an eight-shaped Lissajous curve centered on (5, 3).
        """
        # Get current vehicle state
        px, py, heading, speed = np.array(state_vector, dtype=float).flatten()

        # Get moving target position
        target = target_position(simulation_time)

        # ----------------------------------------------------
        # GEOMETRIC QUANTITIES
        # ----------------------------------------------------
        dx = target[0] - px
        dy = target[1] - py
        distance_to_target = np.hypot(dx, dy)

        # ----------------------------------------------------
        # SPEED REGULATOR
        # ----------------------------------------------------
        maximal_speed = 2.0
        k_distance = 0.8
        desired_speed = min(k_distance * distance_to_target, maximal_speed)

        k_speed = 1.5
        acceleration = k_speed * (desired_speed - speed)

        # ----------------------------------------------------
        # HEADING REGULATOR
        # ----------------------------------------------------
        desired_heading = np.arctan2(dy, dx)

        heading_error = desired_heading - heading
        heading_error = (heading_error + np.pi) % (2.0 * np.pi) - np.pi

        k_heading = 2.0
        heading_rate = k_heading * heading_error

        # ----------------------------------------------------
        # RETURN COMMAND VECTOR
        # ----------------------------------------------------
        return np.array([heading_rate, acceleration], dtype=float)

    def new_mission_is_completed_method(self, state_vector, simulation_time):
        """
        No mission completion criterion: the simulation stops only when the
        maximal simulation duration is reached.
        """
        return False

    # Create instance of DubinsCar class
    dubins_car = DubinsCar()

    # Override the two virtual methods with custom ones
    dubins_car.compute_command_vector = MethodType(new_compute_command_vector_method, dubins_car)
    dubins_car.mission_is_completed = MethodType(new_mission_is_completed_method, dubins_car)

    # Simulation parameters
    duration = 30.0      # seconds
    time_step = 0.01     # seconds

    # Run simulation
    dubins_car.run_simulation(duration, time_step, debug_print=True)

    # Get robot trajectory
    trajectory = np.array(dubins_car.state_vector_trajectory)
    time_vector = np.array(dubins_car.time_trajectory)

    # Reconstruct target trajectory over the same time stamps
    target_trajectory = np.array([target_position(t) for t in time_vector])

    # Display
    plt.figure(figsize=(8, 6))

    # Robot trajectory
    plt.plot(trajectory[:, 0], trajectory[:, 1], label="Dubins car trajectory", linewidth=2, zorder=10)

    # Target trajectory
    plt.plot(target_trajectory[:, 0], target_trajectory[:, 1], "--", label="Target trajectory", linewidth=2)

    # Initial positions
    plt.scatter(trajectory[0, 0], trajectory[0, 1], marker="o", s=60, label="Robot start")
    plt.scatter(target_trajectory[0, 0], target_trajectory[0, 1], marker="x", s=80, label="Target start")

    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Dubins car tracking a moving target on an eight-shaped Lissajous curve")
    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    plt.tight_layout()
    plt.show()

