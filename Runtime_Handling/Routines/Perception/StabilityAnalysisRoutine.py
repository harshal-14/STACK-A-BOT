"""
Routine for analyzing 3D point clouds and finding stable box placements.
"""

import numpy as np
import os
import open3d as o3d

from ..Routine import Routine
from ...Status import Status, Condition
from ....Algorithms.Stability.stability_analyzer import main as stability_main
from ....Algorithms.Stability.stability_analyzer import evaluate_box_placement, find_best_placement

class StabilityAnalysisRoutine(Routine):
    """Routine for analyzing 3D point clouds and finding stable box placements.
    
    This routine processes a 3D point cloud to find horizontal surfaces 
    and determines the most stable position to place a box.
    
    Attributes:
        ply_file_path (str): Path to the PLY file containing the point cloud
        box_size (tuple): Dimensions of the box (width, depth, height) in meters
        visualize (bool): Whether to generate visualizations during processing
        fix_orientation (bool): Whether to automatically fix point cloud orientation
        best_placement (numpy.ndarray): Coordinates of the best placement (x, y, z)
        stability_score (float): Stability score of the best placement (0-1)
    """

    def __init__(self, ply_file_path, box_size=(0.2, 0.15, 0.1), visualize=True, fix_orientation=True):
        """Initialize the stability analysis routine.
        
        Args:
            ply_file_path (str): Path to the PLY file
            box_size (tuple): Dimensions of the box (width, depth, height) in meters
            visualize (bool): Whether to generate visualizations
            fix_orientation (bool): Whether to apply orientation correction
        """
        self.ply_file_path = ply_file_path
        self.box_size = box_size
        self.visualize = visualize
        self.fix_orientation = fix_orientation
        self.best_placement = None
        self.stability_score = 0.0
        self.output_dir = "stability_output"

    # Bridge methods for RoutineScheduler (similar to DUSt3RTestRoutine)
    def _init(self, prev_outputs):
        return self.init(prev_outputs)
        
    def _loop(self):
        return self.loop()
        
    def _end(self):
        return self.end()
        
    def _handle_fault(self, prev_status=None):
        # Create a default Status object if prev_status is None
        if prev_status is None:
            prev_status = Status(
                Condition.Fault,
                err_msg="Unknown fault in routine execution",
                err_type=RuntimeError
            )
        return self.handle_fault(prev_status)

    def init(self, prev_outputs, parameters=None) -> Status:
        """Initialize the stability analysis.
        
        Args:
            prev_outputs (dict): Outputs from previous routines
            parameters (dict, optional): Additional parameters
            
        Returns:
            Status: Success status if initialization is successful
        """
        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        
        # We can use parameters from previous routines if needed
        if parameters is not None:
            if 'ply_file_path' in parameters:
                self.ply_file_path = parameters['ply_file_path']
            if 'box_size' in parameters:
                self.box_size = parameters['box_size']
            if 'output_dir' in parameters:
                self.output_dir = parameters['output_dir']
                
        # Validate input file existence
        if not os.path.exists(self.ply_file_path):
            return Status(
                Condition.Fault,
                err_msg=f"Point cloud file not found: {self.ply_file_path}",
                err_type=FileNotFoundError
            )
            
        print(f"Initialized stability analysis for point cloud: {self.ply_file_path}")
        print(f"Box dimensions: {self.box_size}")
        print(f"Output directory: {self.output_dir}")
            
        return Status(Condition.Success)

    def loop(self) -> Status:
        """Execute the stability analysis.
        
        This is the main processing step that analyzes the point cloud
        and finds the optimal box placement.
        
        Returns:
            Status: Success status when analysis is complete
        """
        try:
            print("Running stability analysis...")
            # Run the stability analysis
            self.best_placement, self.stability_score = stability_main(
                self.ply_file_path,
                box_size=self.box_size,
                visualize=self.visualize,
                fix_orientation=self.fix_orientation
            )
            
            if self.best_placement is None:
                # This is expected when no suitable surfaces are found in the point cloud
                # Rather than treating it as a fault, just mark the routine as complete with no placement
                print("No suitable placement found for the box on any surface.")
                print("Analysis completed without errors, but no stable placement was identified.")
                self.best_placement = None
                self.stability_score = 0.0
                
                # Return Success since the algorithm ran correctly, even though no placement was found
                # This will prevent handle_fault from being called
                return Status(Condition.Success)
                
            print(f"Stability analysis complete. Best placement: {self.best_placement}")
            print(f"Stability score: {self.stability_score:.4f}")
                
            return Status(Condition.Success)
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            return Status(
                Condition.Fault,
                err_msg=f"Error during stability analysis: {str(e)}",
                err_type=type(e)
            )

    def end(self) -> tuple[Status, dict]:
        """Finalize the stability analysis and return results.
        
        Returns:
            tuple: Status and output dictionary with placement results
        """
        # Prepare output dictionary with results
        outputs = {
            'best_placement': self.best_placement.tolist() if self.best_placement is not None else None,
            'stability_score': float(self.stability_score),
            'box_size': self.box_size,
            'output_dir': self.output_dir
        }
        
        # Save results to a text file for easier access
        if self.best_placement is not None:
            results_path = os.path.join(self.output_dir, "placement_results.txt")
            try:
                with open(results_path, "w") as f:
                    f.write(f"Input file: {self.ply_file_path}\n")
                    f.write(f"Box dimensions: {self.box_size[0]} x {self.box_size[1]} x {self.box_size[2]} meters\n")
                    f.write(f"Best placement: [{self.best_placement[0]:.6f}, {self.best_placement[1]:.6f}, {self.best_placement[2]:.6f}]\n")
                    f.write(f"Stability score: {self.stability_score:.6f}\n")
                print(f"Saved results to {results_path}")
            except Exception as e:
                print(f"Warning: Could not save results file: {str(e)}")
        
        print("Stability analysis routine completed successfully.")
        return Status(Condition.Success), outputs

    def handle_fault(self, prev_status) -> tuple[Status, dict]:
        """Handle faults that occur during stability analysis.
        
        Args:
            prev_status (Status): The status from the last ran function
            
        Returns:
            tuple: Status and output dictionary (empty on fault)
        """
        print(f"Stability analysis fault handler called: {prev_status.err_msg}")
        
        # Special handling for "No suitable placement" errors
        if "No suitable placement" in prev_status.err_msg or "NO SUITABLE SURFACES FOUND" in prev_status.err_msg:
            print("This is expected when no suitable surfaces can be found in the point cloud.")
            print("The point cloud may be too sparse, too noisy, or not contain horizontal surfaces.")
            
            # Return an empty dictionary but with some useful information
            return prev_status, {
                'best_placement': None,
                'stability_score': 0.0,
                'box_size': self.box_size,
                'output_dir': self.output_dir,
                'error': prev_status.err_msg
            }
        
        # Return empty dictionary as no valid results were produced
        return prev_status, {}