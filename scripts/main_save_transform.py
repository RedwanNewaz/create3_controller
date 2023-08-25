import subprocess
import os
import re
import argparse

class DFA:
    def __init__(self):
        self.state = 0
        self.accept = 2
        self.data = []

    def __call__(self, data):
        N = len(data)
        if self.state == 0 and N == 3:
            self.state = 1
            self.data.append(data)
        if self.state == 1 and N == 4:
            self.state = 2
            self.data.append(data)

    def get(self):
        assert len(self.data) == 2
        return self.data[0] + self.data[1]
    def reset(self):
        self.data = []
    def isAccepted(self):
        if self.state == 2:
            self.state = 0
            return True
        return False


def execute_command(command):
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
        shell=True
    )
    pattern = r'\[([-0-9., ]+)\]'
    dfa = DFA()
    while True:
        output = process.stdout.readline()
        if output == '' and process.poll() is not None:
            break
        if output:
            matches = re.findall(pattern, output)
            # Check if there are any matches
            if matches:
                # Get the first match (assuming there's only one match in the text)
                matched_string = matches[0]

                # Split the matched string into a list of strings
                number_strings = matched_string.split(',')

                # Convert the list of strings to a set of float numbers
                number_list = list(float(num) for num in number_strings)

                # Print the set of float numbers
                dfa(number_list)
                if dfa.isAccepted():
                    yield dfa.get()
                    dfa.reset()
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--frm", type=str, default="nexigo_cam", help="From which camera frame?")
    parser.add_argument("--to", type=str, default="tag36h11:32", help="Family name with tag id for tag")
    parser.add_argument("--n", type=int, default=15, help="Number of samples")
    parser.add_argument("--save", action="store_true", help="Save result as csv file")
    parser.add_argument("-v", "--verbose", action="store_false", help="Show results?")
    args = parser.parse_args()

    command_to_execute = "ros2 run tf2_ros tf2_echo {} {}".format(args.frm, args.to)
    print(command_to_execute)

    data = []
    for i, val in enumerate(execute_command(command_to_execute)):

        # show results
        if args.verbose:
            print(i + 1 , val)

        # save result
        if args.save:
            data.append(val)

        # check termination
        if i >= args.n - 1:
            break
    if args.save:
        outfile = "tf_{}_{}.csv".format(args.frm, args.to)
        with open(outfile, 'w+') as fp:
            for tf in data:
                line = ", ".join(map(str, tf)) + "\n"
                fp.write(line)
