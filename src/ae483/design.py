"""Tools for exporting a control design to the drone.

The idea is that one thing - the *structure* of the controller - determines
everything else:

  * the names of the gains,
  * the C code that declares them and makes them parameters,
  * the C code that computes the inputs from the gains,
  * the JSON file of gain values that flight.py sends to the drone.

The structure comes from the model (which states are coupled to which inputs),
not from your choice of Q and R. Q and R only change the numbers.
"""

import json
import numpy as np
from datetime import datetime
from pathlib import Path
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components


# The name of the parameter group that holds gains. Everything in this group is
# a gain, which is what lets flight.py ask the drone which gains it expects.
GAIN_GROUP = 'ae483gain'

# A parameter name is sent as "group.name" and the total must fit in this many
# characters (see the comment above the parameter groups in controller_ae483.c).
MAX_TOTAL_LENGTH = 28
MAX_NAME_LENGTH = MAX_TOTAL_LENGTH - len(GAIN_GROUP) - 1


def gain_structure(A, B, s, i, tol=1e-12):
    """
    Return a dict that says which states each input's gains multiply.

    A, B are the matrices of the linearized model. s and i are lists of symbolic
    states and inputs. The result is derived from which states and inputs are
    coupled in the model - so it does not depend on Q and R, and it does not
    depend on which gains happen to come out near zero for a particular design.

    For the usual AE483 model this returns the four decoupled subsystems:

        tau_x -> p_y, phi, v_y, w_x
        tau_y -> p_x, theta, v_x, w_y
        tau_z -> psi, w_z
        f_z   -> p_z, v_z
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    n, m = len(s), len(i)

    # Build a graph whose nodes are the states and the inputs, with an edge
    # wherever the model couples two of them, then find connected components.
    M = np.zeros((n + m, n + m), dtype=bool)
    M[:n, :n] = np.abs(A) > tol
    M[:n, n:] = np.abs(B) > tol
    M |= M.T
    _, label = connected_components(csr_matrix(M), directed=False)

    return {
        _name_of(i[k]): [_name_of(s[j]) for j in range(n) if label[j] == label[n + k]]
        for k in range(m)
    }


def gain_name(input_name, state_name):
    """The name of the gain that multiplies a state in an input's equation."""
    return f'k_{input_name}_{state_name}'


def equilibrium_name(input_name):
    """The name of the equilibrium value of an input."""
    return f'eq_{input_name}'


def gain_names(structure):
    """Every parameter name implied by a structure, in a fixed order."""
    names = []
    for input_name, state_names in structure.items():
        names.append(equilibrium_name(input_name))
        for state_name in state_names:
            names.append(gain_name(input_name, state_name))
    _check_name_lengths(names)
    return names


def export_declarations(structure):
    """Print the C code that declares each gain. Paste this into
    controller_ae483.c, near the top, with the other variables."""
    print('// Gains (these are set by flight.py - do not edit the values here)')
    for name in gain_names(structure):
        print(f'static float {name} = 0.0f;')


def export_parameter_block(structure):
    """Print the C code that makes each gain a parameter. Paste this into
    controller_ae483.c, at the bottom, with the other parameter groups.

    The layout matches the parameter and log groups that are already in that
    file. The group name, and the ruler comments above it, start just after
    "PARAM_GROUP_START(". Each parameter name lines up with the word "name" in
    the second ruler comment, and each pointer sits 25 characters after that.
    """
    group_column = len('PARAM_GROUP_START(')
    name_column = group_column + len(GAIN_GROUP) + 1   # just past "group."
    pointer_column = name_column + 25
    ruler = ''.join(str((n + 1) % 10) for n in range(MAX_TOTAL_LENGTH))

    print(f'//{" " * (group_column - 2)}{ruler} <-- max total length')
    print(f'//{" " * (group_column - 2)}group{" " * max(1, len(GAIN_GROUP) - len("group"))}.name')
    print(f'PARAM_GROUP_START({GAIN_GROUP})')
    for name in gain_names(structure):
        print(f'{"PARAM_ADD(PARAM_FLOAT,":<{name_column}}'
              f'{name + ",":<{pointer_column - name_column}}&{name})')
    print(f'PARAM_GROUP_STOP({GAIN_GROUP})')


def export_controller(structure, s_with_des):
    """
    Print the C code that computes each input from the gains. Paste this into
    controller_ae483.c, inside controllerAE483().

    The gains are named rather than written out as numbers, so this code does
    not change when you redesign your controller - only the numbers sent by
    flight.py change. You only need to paste this again if the *structure*
    changes, e.g., when you add a model of time delay or integral action.
    """
    s_with_des = [_name_of(x) for x in s_with_des]
    for input_name, state_names in structure.items():
        terms = []
        for state_name in state_names:
            if state_name in s_with_des:
                error = f'({state_name} - {state_name}_des)'
            else:
                error = state_name
            terms.append(f'{gain_name(input_name, state_name)} * {error}')
        body = ' + '.join(terms)
        print(f'{input_name} = {equilibrium_name(input_name)} - ({body});')


def export_gains(K, structure, s, i, i_eq, filename='gains.json', verbose=True):
    """
    Write the gain values to a JSON file for flight.py to send to the drone.

    Every gain in the structure is written, including those that came out at or
    near zero. That matters: if a gain were left out, the drone would keep
    whatever value it had from your previous flight.
    """
    K = np.asarray(K, dtype=float)
    s_name = [_name_of(x) for x in s]
    i_name = [_name_of(x) for x in i]

    gains = {}
    for input_name, state_names in structure.items():
        row = i_name.index(input_name)
        gains[equilibrium_name(input_name)] = float(i_eq[row])
        for state_name in state_names:
            col = s_name.index(state_name)
            gains[gain_name(input_name, state_name)] = float(K[row, col])

    _check_name_lengths(list(gains.keys()))

    contents = {
        'created': datetime.now().isoformat(timespec='seconds'),
        'structure': {k: list(v) for k, v in structure.items()},
        'gains': gains,
    }

    with open(filename, 'w') as f:
        json.dump(contents, f, indent=2, sort_keys=False)

    if verbose:
        print(f'Wrote {len(gains)} gains to {filename} at {contents["created"]}')
        width = max(len(name) for name in gains)
        for name, value in gains.items():
            print(f'  {name:{width}s} = {value: .8f}')


def gains_as_text(gains, decimals=4):
    """
    Format control gains compactly, one line per input, in the same order they
    appear in the equations of your controller.
    """
    # Work out the labels and column widths first, so that every line lines up.
    # Each gain is named k_<input>_<state>, and the name of the input is known
    # from the eq_<input> entry that comes just before its gains - so there is
    # no need to guess where the name of the input ends and the state begins.
    labels = {}
    name_of_input = None
    for name in gains:
        if name.startswith('eq_'):
            name_of_input = name[len('eq_'):]
            labels[name] = 'eq'
        else:
            prefix = f'k_{name_of_input}_'
            labels[name] = name[len(prefix):] if name.startswith(prefix) else name
    label_width = max(len(x) for x in labels.values())
    input_width = max(len(n[len('eq_'):]) for n in gains if n.startswith('eq_'))

    lines, parts, name_of_input = [], [], None
    for name, value in gains.items():
        if name.startswith('eq_'):
            if name_of_input is not None:
                lines.append(f'{name_of_input:>{input_width}s}:  ' + '  '.join(parts))
            name_of_input = name[len('eq_'):]
            parts = []
        parts.append(f'{labels[name]:>{label_width}s} {value:{decimals + 4}.{decimals}f}')
    if name_of_input is not None:
        lines.append(f'{name_of_input:>{input_width}s}:  ' + '  '.join(parts))
    return '\n'.join(lines)


def show_gains(filename, decimals=4):
    """
    Print the control gains that were used for a flight.
    """
    with open(Path(filename), 'r') as f:
        data = json.load(f)
    gains = data.get('gains', None)
    created = data.get('gains_created', None)

    if gains:
        print(f'Control gains, designed {created}:' if created else 'Control gains:')
        print(gains_as_text(gains, decimals=decimals))
    elif data.get('use_controller', False):
        print('WARNING: This flight was set up to use your controller, but no gains')
        print('were recorded. That means the gains never reached the drone, so it')
        print('would not have flown. Check the output of flight.py for an error.')
    else:
        print('This flight used the default controller, so there were no gains.')


def show_controller(K, s, i, s_with_des, i_eq,
                    decimals=8,
                    suffix='',
                    line_ending=''):
    """
    Print the control law with its gains written out as numbers.

    This is for reading and for your report - it shows what your design
    actually is, and which gains are small enough to ignore. It is NOT what you
    paste into controller_ae483.c: that code uses named gains and comes from
    export_controller(), and the numbers themselves are sent by flight.py.

    K is a gain matrix, of size m x n
    s is a list of states as symbolic variables, of length n
    i is a list of inputs as symbolic variables, of length m
    s_with_des is a list of states that have desired values, as
        symbolic variables - if there are no such states, then
        this should be an empty list []
    i_eq is a list of equilibrium values of inputs, of length m
    decimals is the number of decimals to include when printing
        each value
    suffix is the character (if any) to print after each number,
        for example 'f' to indicate a "float" when exporting to C
    line_ending is the character (if any) to print after each
        line, for example ';' when exporting to C
    """
    s_name = [_name_of(scur) for scur in s]
    i_name = [_name_of(icur) for icur in i]
    des_name = [_name_of(x) for x in s_with_des]
    for row in range(len(i_name)):
        input_string = ''
        for col in range(len(s_name)):
            k = K[row, col]
            if not np.isclose(k, 0.):
                if (k < 0) and input_string:
                    input_string += ' +'
                if s_name[col] in des_name:
                    n = f'({s_name[col]} - {s_name[col]}_des)'
                else:
                    n = s_name[col]
                input_string += f' {-k:.{decimals}f}{suffix} * {n}'
        if not np.isclose(i_eq[row], 0.):
            if (i_eq[row] > 0) and input_string:
                input_string += ' +'
            input_string += f' {i_eq[row]:.{decimals}f}{suffix}'
        print(f'{i_name[row]} ={input_string}{line_ending}')


def export_power_distribution(Pinv,
                              i=None,
                              m=None,
                              limiter='limitUint16',
                              decimals=1,
                              suffix='',
                              line_ending=''):
    """
    Print the C code that maps inputs to motor power commands.

    Pinv is a 4 x 4 matrix that maps inputs to motor power commands
    i is a list of inputs as symbolic variables or strings (by default,
        this list is ['tau_x', 'tau_y', 'tau_z', 'f_z'])
    m is a list of motor power commands as symbolic variables or strings
        (by default, this list is ['m_1', 'm_2', 'm_3', 'm_4'])
    limiter is the name of the function to apply that ensures each
        motor power command is valid (i.e., an integer within bounds),
        for example "limitUint16" when exporting to C
    decimals is the number of decimals to include when printing
        each value
    suffix is the character (if any) to print after each number,
        for example 'f' to indicate a "float" when exporting to C
    line_ending is the character (if any) to print after each
        line, for example ';' when exporting to C
    """
    if i is None:
        i = ['tau_x', 'tau_y', 'tau_z', 'f_z']
    if m is None:
        m = ['m_1', 'm_2', 'm_3', 'm_4']
    i_name = [_name_of(icur) for icur in i]
    m_name = [_name_of(mcur) for mcur in m]
    for row in range(len(m_name)):
        input_string = ''
        for col in range(len(i_name)):
            k = Pinv[row, col]
            if not np.isclose(k, 0.):
                if (k > 0) and input_string:
                    input_string += ' +'
                n = i_name[col]
                input_string += f' {k:.{decimals}f}{suffix} * {n}'
        print(f'{m_name[row]} = {limiter}({input_string} ){line_ending}')


def _name_of(symbol):
    """Accept either a sympy symbol or a plain string."""
    return symbol.name if hasattr(symbol, 'name') else str(symbol)


def _check_name_lengths(names):
    too_long = [name for name in names if len(name) > MAX_NAME_LENGTH]
    if too_long:
        raise ValueError(
            f'These parameter names are longer than {MAX_NAME_LENGTH} characters, '
            f'which is the most that fits in "{GAIN_GROUP}.<name>": {too_long}'
        )
