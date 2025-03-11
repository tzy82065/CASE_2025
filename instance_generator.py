import os

def process_scen_file(input_path, output_path, agents, file_num):
    # Create output directory if not exists
    os.makedirs(output_path, exist_ok=True)
    
    try:
        # Open and read input file
        with open(input_path, 'r') as infile:
            # Skip first line (version 1)
            next(infile)
            
            # Read required number of lines
            lines = []
            for _ in range(agents):
                line = next(infile)
                # Split line by tabs and get coordinates (5th-8th columns)
                coords = line.strip().split('\t')[4:8]
                # Join coordinates with comma
                lines.append(','.join(coords))
        
        # Create output file
        out_filename = f"warehouse-10-20-10-2-2-{agents}-{file_num}.txt"
        out_filepath = os.path.join(output_path, out_filename)
        
        # Write to output file
        with open(out_filepath, 'w') as outfile:
            # Write fixed header
            outfile.write("map_file=warehouse-10-20-10-2-2.map\n")
            outfile.write(f"agents={agents}\n")
            outfile.write("seed=0\n")
            outfile.write("random_problem=0\n")
            outfile.write("max_timestep=1000\n")
            outfile.write("max_comp_time=5000\n")
            
            # Write coordinate lines
            outfile.write('\n'.join(lines))
            
    except Exception as e:
        print(f"Error processing file {input_path}: {str(e)}")

def main():
    # Base paths
    base_input_path = "../MAPF_T-main/instance/warehouse-10-20-10-2-2/scen-random"
    base_output_path = "../pibt2/instances/test/warehouse-10-20-10-2-2"
    
    # Agent counts to process
    agent_counts = [550,600,650,700,750,800,850,900,950,1000]
    
    # Process each agent count
    for agents in agent_counts:
        # Create agent-specific output directory
        output_path = os.path.join(base_output_path, str(agents))
        
        # Process 25 scenario files
        for i in range(1, 26):
            input_file = os.path.join(base_input_path, f"warehouse-10-20-10-2-2-random-{i}.scen")
            process_scen_file(input_file, output_path, agents, i)
            
        print(f"Processed {agents} agents scenario")

if __name__ == "__main__":
    main()