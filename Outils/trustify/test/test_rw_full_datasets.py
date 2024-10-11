"""
Testing loading of various dataset examples into the datamodel generated by trustify.
"""

import os
import unittest

from test.reference_data import *
import trustify.misc_utilities as mutil
from trustify.misc_utilities import ClassFactory, logger
from trustify.trust_parser import TRUSTParser, TRUSTStream

########################################################################################
class TestCase(unittest.TestCase, mutil.UnitUtils):
    """
    Testing loading of various complete dataset examples into the datamodel generated by trustify.
    """

    _test_dir = os.path.abspath(os.path.dirname(__file__))
    _models = []

    def generic_simple(self, data_ex, typ_nam, slot="full", fnam="??"):
        """ Test a single entry of a dataset """
        # Generate if needed - the full thing here!!
        self.generate_python_and_import(slot)
        self.mod = self._TRUG[slot]
        # Parse the TRUST data set provided in arg
        tp = TRUSTParser()
        tp.tokenize(data_ex)
        stream = TRUSTStream(parser=tp, file_nam=fnam)
        trust_cls = ClassFactory.GetParserClassFromName(typ_nam)
        return trust_cls.ReadFromTokens(stream)

    def generic_test(self, data_ex, fnam="??"):
        """ Generic test method taking an entire dataset (=several entries) and testing it against a frozen version of the TRAD2 """
        return self.generic_simple(data_ex, "Dataset", slot="full", fnam=fnam)

    def generic_custom_test(self, data_ex, fnam="??"):
        """ Same as above but with a . """
        return self.generic_simple(data_ex, "Dataset", slot="custom", fnam=fnam)

    def load_file(self, fName):
        """ Open a test file located in subfolder 'datasets' and return its content as a string. """
        dr = self._test_dir + "/datasets"
        with open(dr + "/" + fName) as f:
            # rstrip() to get rid of any end of line, blanks, etc ... after the 'end' keyword
            return f.read().rstrip()

    def setDimension(self, dim):
        self.generate_python_and_import("full")
        self.mod = self._TRUG["full"]
        self.mod.Dimension_Parser._DIMENSION = dim

#############################################

    def test_small_ds(self):
        """ Almost the same as test_complete_dataset in test_rw_elementary, but this time:
          - with the full TRAD_2 file
          - with the domain attribute set, which was not there in the simpler case
        """
        # self.__class__._NO_REGENERATE = True

        data_ex = """
        # Some stupid test #
        champ_uniforme gravite
        nom coucou
        read gravite 2 28 32   # Keyword with no brace #
        read coucou toto
        lire_MED
          {
             domain dom
             mesh ze_mesh_name
             file a/complicated/path/to.med
             exclude_groups 2 toto titi
             convertAllToPoly
          }"""
        res = self.generic_test(data_ex)
        self.assertEqual(len(res.entries), 5)
        # Build expected value
        exp = buildMinimalExpec(self.mod)
        exp[4].domain = "dom"
        # Test
        for i in range(5):
            self.assertEqual(exp[i], res.entries[i])
        # Test write out:
        s = ''.join(res.toDatasetTokens())
        self.assertTrue(mutil.check_str_equality(s, data_ex).ok)

    def test_probes(self):
        """ Probes ... what a pain ... deserve their own unit test. """
        # self.__class__._NO_REGENERATE = True

        # Force dimension (normally comes from keyword 'dimension' in dataset)
        self.setDimension(2)

        # One probe (type 'sonde' in TRAD_2):
        data_ex = "sonde_pression pression periode 0.005 points 2 0.13 0.105 0.13 0.115"
        res = self.generic_simple(data_ex, "Sonde")
        S = self.mod.Sonde
        exp = buildSondesExpec(self.mod)
        self.assertEqual(exp, res)
        # Test writing out:
        s = ''.join(res.toDatasetTokens())
        self.assertTrue(mutil.check_str_equality(s, data_ex).ok)
        # Try changing value:
        res.type.points[0].pos[1] = 0.123
        news = "sonde_pression pression periode 0.005 points 2 0.13 0.123 0.13 0.115"
        s = ''.join(res.toDatasetTokens())
        self.assertTrue(mutil.check_str_equality(s, news).ok)

        # Reset dimension!
        self.setDimension(-1)

    def test_bords(self):
        """ Bords (in 'mailler') are also tricky """
        # self.__class__._do_not_regenerate = True
        data_ex = "Bord periox X =   0.  0. <= Y <= # up bound # 2.0"

        # Force dimension (normally comes from keyword 'dimension' in dataset)
        self.setDimension(2)

        # Replicate hack performed in 'hacks.py' when dimension is read in the dataset:
        S = self.mod.Bord
        fld = S.model_fields["defbord"]
        fld.annotation =  self.mod.Defbord_2

        res = self.generic_simple(data_ex, "Bord")
        exp = buildBordExpec(self.mod)
        self.assertEqual(exp, res)
        # Test writing out:
        s = ''.join(res.toDatasetTokens())
        self.assertTrue(mutil.check_str_equality(s, data_ex).ok)
        # Try changing value:
        res.nom = "bip"
        res.defbord.pos = 24.2
        news = """Bord bip X = 24.2  0. <= Y <= # up bound # 2.0"""
        s = ''.join(res.toDatasetTokens())
        self.assertTrue(mutil.check_str_equality(s, news).ok)
        # Reset dimension
        self.setDimension(-1)

    def test_bloc_lecture(self):
        """ Testing bloc_lecture via 'solveur_pression' """
        # self.__class__._do_not_regenerate = True
        data_ex = """{
           solveur_pression petsc cholesky {   nImp c est un blOc leCtuRe }
           convection { negligeable }
        }"""
        # (note how we didn't put the 'navier_stokes_standard' in the dataset, because of what is done
        # in hacks.py, seting _readType to False)
        res = self.generic_simple(data_ex, "Navier_stokes_standard")
        exp = buildSolveurPressionExpec(self.mod)
        self.assertEqual(exp, res.solveur_pression)
        # Test writing out:
        s = ''.join(res.toDatasetTokens())
        self.assertTrue(mutil.check_str_equality(s, data_ex).ok)
        # Try changing value:
        res.solveur_pression.option_solveur.bloc_lecture = "{  taDa tudu }"
        news = """ petsc cholesky {  taDa tudu }"""
        s = ''.join(res.solveur_pression.toDatasetTokens())
        self.assertTrue(mutil.check_str_equality(s, news).ok)

    def test_complex_list(self):
        """ Testing complex lists - TODO duplicate of test_rw_elementary.test_complex_list?"""
        data_ex = """
        dom_solide
        {
            Pave # coucou # Cavite1 {  # nothing # }   {  } ,
            Pave # hello   #   Cavite2 { } { }
        }"""
        res = self.generic_simple(data_ex, "Mailler")
        # Test writing out:
        s = ''.join(res.toDatasetTokens())
        self.assertTrue(mutil.check_str_equality(s, data_ex).ok)
        # Try changing value (delete last item in list, and change name)
        res.bloc.pop()
        res.bloc[0].name = "toto"
        news = """
        dom_solide
        {
            Pave toto {  # nothing # }   {  }
        }"""
        s = ''.join(res.toDatasetTokens())
        self.assertTrue(mutil.check_str_equality(s, news).ok)

############################################################
## From here on, full datasets from subfolder 'datasets'
##   We don't build full expected value in Python anymore
##   (too much work ...) and simply test equality when writing out.
############################################################
    def test_ds_upwind(self):
        """ Dataset: upwind_simplified.data """
        # self.__class__._do_not_regenerate = True

        data_ex = self.load_file("upwind_simplified.data")
        res = self.generic_test(data_ex)
        # Test write out:
        s = ''.join(res.toDatasetTokens())
        self.assertTrue(mutil.check_str_equality(s, data_ex).ok)
        # # Test changing one bit:
        # tmp = res.get("pb").post_processing.sondes.pop()
        # tmp._parentAsAttribute = None
        # res.get("pb").post_processing.sondes.insert(0, tmp)
        # one_prob.type.points[0].pos[1] = 0.123
        # s = ''.join(res.toDatasetTokens())
        # print(s)

    def test_ds_canal(self):
        """ Dataset: Canal_perio_VEF_2D.data """
        data_ex = self.load_file("Canal_perio_VEF_2D.data")
        res = self.generic_test(data_ex)
        # Test write out:
        s = ''.join(res.toDatasetTokens())
        self.assertTrue(mutil.check_str_equality(s, data_ex).ok)

    def test_ds_diff_impl(self):
        """ Dataset: diffusion_implicite_jdd6.data """
        data_ex = self.load_file("diffusion_implicite_jdd6.data")
        res = self.generic_test(data_ex)
        # Test write out:
        s = ''.join(res.toDatasetTokens())
        self.assertTrue(mutil.check_str_equality(s, data_ex).ok)

    def test_ds_distance_paroi(self):
        """ Dataset:  distance_paroi_jdd1.data
        which used various <inherited> things (like 'ice' solver) and 'bloc_lecture'
        """
        data_ex = self.load_file("distance_paroi_jdd1.data")
        res = self.generic_test(data_ex)
        # Test write out:
        s = ''.join(res.toDatasetTokens())
        self.assertTrue(mutil.check_str_equality(s, data_ex).ok)

    def test_single_ds(self):
        """ tmp single dataset """
        # self.__class__._do_not_regenerate = True

        e = "/export/home/adrien/Projets/TRUST/TRUST_LOCAL_fourth/tests/Reference/PCR/PCR.data"
        e = "/export/home/adrien/Projets/TRUST/TRUST_LOCAL_fourth/tests/GPU/VDF_AMGX/VDF_AMGX.data"
        e ="/export/home/adrien/Projets/TRUST/TRUST_LOCAL_fourth/tests/Reference/Kernel_Extrusion_en20/Kernel_Extrusion_en20.data"
        e = "/export/home/adrien/Projets/TRUST/TRUST_LOCAL_fourth/tests/Reference/Pertes_d_charges_CF_VDF/Pertes_d_charges_CF_VDF.data"
        e = "/export/home/adrien/Projets/TRUST/TRUST_LOCAL_fourth/tests/Verification/Part_Ss_Dom_Union_jdd2/Part_Ss_Dom_Union_jdd2.data"
        d = os.path.dirname(e)
        bas = os.path.split(d)[-1]
        fNam = os.path.join(d, bas + ".data")
        with open(fNam) as f:
            data_ex = f.read()
        # printAdrien("'" + data_ex + "'")
        res = self.generic_custom_test(data_ex, fnam=e)
        # Test write out:
        s = ''.join(res.toDatasetTokens())
        data_ex_p = mutil.prune_after_end(data_ex)
        self.assertTrue(mutil.check_str_equality(s, data_ex_p).ok)

    def test_all_trust_ds(self):
        """ Test **all** TRUST datasets """
        import glob

        trust_root = os.environ.get("TRUST_ROOT", None)
        if trust_root is None:
            raise Exception("TRUST_ROOT not defined! Have you sourced TRUST?")
        tst_dir = os.path.join(trust_root, "tests")
        # Find all datasets:
        pattern = os.path.abspath(os.path.join(tst_dir, "**/*.lml.gz"))
        g = glob.glob(pattern, recursive=True)
        with open("/nfs/home/catA/ab205030/failed_trustify.txt") as fl:
            short_lst = [s.strip() for s in fl.readlines()]
        ko = []
        for i, e in enumerate(g[:]):
            # Extract dir name, and build dataset file name
            d = os.path.dirname(e)
            bas = os.path.split(d)[-1]
            fNam = os.path.join(d, bas + ".data")
            print("%d/%d -- %s" % (i+1, len(g), fNam), end="\r")
            if fNam in short_lst:
                continue
            with open(fNam) as f:
                data_ex = f.read()
            try:
                res = self.generic_custom_test(data_ex, fnam=fNam)
                # Test write out:
                s = ''.join(res.toDatasetTokens())
                data_ex_p = mutil.prune_after_end(data_ex)
                self.assertTrue(mutil.check_str_equality(s, data_ex_p).ok)
            except Exception as e:
                ko.append(fNam)
                print(f"   Dataset KO: {fNam}")
                print("    -> KO :-( %s" % e)
        print("\n".join(ko))
        print(len(ko))

if __name__ == '__main__':
    unittest.main()
