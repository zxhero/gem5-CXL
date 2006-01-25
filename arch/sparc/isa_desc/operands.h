def operand_types {{
    'sb' : ('signed int', 8),
    'ub' : ('unsigned int', 8),
    'shw' : ('signed int', 16),
    'uhw' : ('unsigned int', 16),
    'sw' : ('signed int', 32),
    'uw' : ('unsigned int', 32),
    'sdw' : ('signed int', 64),
    'udw' : ('unsigned int', 64),
    'sf' : ('float', 32),
    'df' : ('float', 64),
    'qf' : ('float', 128)
}};

def operands {{
    # Int regs default to unsigned, but code should not count on this.
    # For clarity, descriptions that depend on unsigned behavior should
    # explicitly specify '.uq'.
    'Rd': IntRegOperandTraits('udw', 'RD', 'IsInteger', 1),
    'Rs1': IntRegOperandTraits('udw', 'RS1', 'IsInteger', 2),
    'Rs2': IntRegOperandTraits('udw', 'RS2', 'IsInteger', 3),
    #'Fa': FloatRegOperandTraits('df', 'FA', 'IsFloating', 1),
    #'Fb': FloatRegOperandTraits('df', 'FB', 'IsFloating', 2),
    #'Fc': FloatRegOperandTraits('df', 'FC', 'IsFloating', 3),
    'Mem': MemOperandTraits('udw', None,
                            ('IsMemRef', 'IsLoad', 'IsStore'), 4)
    #'NPC': NPCOperandTraits('uq', None, ( None, None, 'IsControl' ), 4),
    #'Runiq': ControlRegOperandTraits('uq', 'Uniq', None, 1),
    #'FPCR':  ControlRegOperandTraits('uq', 'Fpcr', None, 1),
    # The next two are hacks for non-full-system call-pal emulation
    #'R0':  IntRegOperandTraits('uq', '0', None, 1),
    #'R16': IntRegOperandTraits('uq', '16', None, 1)
}};
